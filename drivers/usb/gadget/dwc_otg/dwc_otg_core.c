#include "dwc_otg_core.h"

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/delay.h>

#include "dwc_otg_hw.h"

/**
 * dwc_otg_core_init_eps
 *
 * This is a helper function for dwc_otg_core_init that
 * sets up the EP structures.
 */
int dwc_otg_core_init_eps(dwc_otg_core_t *_core)
{
	int i;

	// Clear EP data structure (this sets them all as inactive).
	memset(&_core->endpoints, sizeof(_core->endpoints), 0);

	for(i = 0; i < MAX_EPS_CHANNELS; i++)
	{
		uint32_t ret = 0;

		_core->in_ep_registers[i] = (dwc_otg_dev_in_ep_regs_t*)(((uint8_t*)_core->registers) + DWC_OTG_IN_EP_OFFSET + (DWC_OTG_IN_EP_SIZE * i));
		_core->out_ep_registers[i] = (dwc_otg_dev_out_ep_regs_t*)(((uint8_t*)_core->registers) + DWC_OTG_OUT_EP_OFFSET + (DWC_OTG_OUT_EP_SIZE * i));

		// Setup Endpoint
		_core->endpoints[i].num = i;
		_core->endpoints[i].speed = 0;
		_core->endpoints[i].active = 0;
		_core->endpoints[i].in_registers = _core->in_ep_registers[i];
		_core->endpoints[i].out_registers = _core->out_ep_registers[i];
		INIT_LIST_HEAD(&_core->endpoints[i].queue_pointer); // Technically not required, keeps debugging clean though.
		INIT_LIST_HEAD(&_core->endpoints[i].transfer_queue);
		spin_lock_init(&_core->endpoints[i].lock);

		ret = snprintf(_core->endpoints[i].name, DWC_OTG_EP_MAX_NAME_SIZE, "ep%d", i);
		if(ret > DWC_OTG_EP_MAX_NAME_SIZE)
		{
			DWC_WARNING("EP name buffer too small.\n");
		}

		/*
		 * This is where we would check the direction,
		 * but it seems that upon further inspection of the
		 * documentation floating around on the internet,
		 * hwcfg1 does in fact _not_ contain the EP
		 * capabilities, but in fact the _current_ ep 
		 * layout. -- Ricky26
		 */
		if(i < _core->num_eps)
			_core->endpoints[i].exists = 1;

		if(i == 0)
			_core->endpoints[i].speed = DWC_OTG_FULL_SPEED;
	}

	return 0;
}

/**
 * dwc_otg_core_init
 *
 * This function initialises the hardware and the
 * structure specified by _core.
 */
int dwc_otg_core_init(dwc_otg_core_t *_core, int _irq, void *_regs, void *_phy)
{
	int i;
	int ret = 0;
	dctl_data_t dctl = { .d32 = 0 };
	pcgcctl_data_t pcgcctl = { .d32 = 0 };

	DWC_VERBOSE("%s(%p, %d, %p, %p)\n", __func__, _core, _irq, _regs, _phy);

	if(!_core)
	{
		DWC_ERROR("%s called with null core pointer.\n", __func__);
		return -EINVAL;
	}

	// Clear core structure.
	// (This should have already been done by the driver interface,
	//  but better safe than sorry.)
	memset(_core, sizeof(dwc_otg_core_t), 0);

	// Initialise internal variables
	INIT_LIST_HEAD(&_core->ep_transfer_queue);

	// Copy parameters into core.
	_core->registers = (dwc_otg_core_global_regs_t*)_regs;
	_core->phy_registers = (dwc_otg_phy_regs_t*)_phy;
	_core->device_registers = (dwc_otg_device_global_regs_t*)(((uint8_t*)_regs) + DWC_OTG_DEVICE_OFFSET);
	_core->pcgcctl = (uint32_t*)(((uint8_t*)_regs) + DWC_OTG_PCGCCTL_OFFSET);

	// Initialise hardware.
	dwc_otg_hw_power(_core, 0); // Make sure everything is reset, incase this
								// hardware was used in the bootloader, for example.
	dwc_otg_hw_power(_core, 1);

	// Read Hardware Configuration Registers
	_core->hwcfg1.d32 = dwc_otg_read_reg32(&_core->registers->ghwcfg1);
	_core->hwcfg2.d32 = dwc_otg_read_reg32(&_core->registers->ghwcfg2);
	_core->hwcfg3.d32 = dwc_otg_read_reg32(&_core->registers->ghwcfg3);
	_core->hwcfg4.d32 = dwc_otg_read_reg32(&_core->registers->ghwcfg4);

	_core->num_eps = _core->hwcfg2.b.num_dev_ep + 1;
	DWC_DEBUG("%d endpoints detected.\n", _core->num_eps);

	// Initialise EP structures
	dwc_otg_core_init_eps(_core);
	
	// Send soft disconnect
	dctl.b.sftdiscon = 1;
	dwc_otg_modify_reg32(&_core->device_registers->dctl, 0, dctl.d32);
	msleep(4);

	// Power on core.
	dwc_otg_write_reg32(_core->pcgcctl, pcgcctl.d32);
	udelay(100);

	if(_core->phy_registers)
	{
		phypwr_data_t phypwr = { .d32 = 0 };	
		phyclk_data_t phyclk = { .d32 = 0 };
		rstcon_data_t rstcon = { .d32 = 0 };

		// Power on PHY, if we have one.
		dwc_otg_write_reg32(&_core->phy_registers->phypwr, phypwr.d32);
		udelay(10);

		// Select 48Mhz
		phyclk.b.clksel = DWC_OTG_PHYCLK_CLKSEL_48MHZ;
		dwc_otg_modify_reg32(&_core->phy_registers->phyclk, DWC_OTG_PHYCLK_CLKSEL_MASK, phyclk.d32);

		// Reset PHY
		rstcon.b.physwrst = 1;
		dwc_otg_modify_reg32(&_core->phy_registers->phyclk, 0, rstcon.d32);
		udelay(20);
		dwc_otg_modify_reg32(&_core->phy_registers->phyclk, rstcon.d32, 0);
		msleep(1);
	}

	dwc_otg_core_soft_reset(_core);

	// Clear soft disconnect
	dwc_otg_modify_reg32(&_core->device_registers->dctl, dctl.d32, 0);
	msleep(4);

	// Clear interrupts
	dwc_otg_write_reg32(&_core->in_ep_registers[_core->num_eps]->diepint, 0xffffffff); // No idea why this is here, reverse-engineered from iBoot.
	dwc_otg_write_reg32(&_core->out_ep_registers[_core->num_eps]->doepint, 0xffffffff);

	for(i = 0; i < _core->num_eps; i++)
	{
		dwc_otg_write_reg32(&_core->in_ep_registers[i]->diepint, 0xffffffff);
		dwc_otg_write_reg32(&_core->out_ep_registers[i]->doepint, 0xffffffff);
	}

	dwc_otg_write_reg32(&_core->registers->gintmsk, 0);
	dwc_otg_write_reg32(&_core->device_registers->diepmsk, 0);
	dwc_otg_write_reg32(&_core->device_registers->doepmsk, 0);

	// Register IRQ.
	ret = request_irq(_irq, dwc_otg_core_irq, IRQF_SHARED, DWC_OTG_DRIVER_NAME, _core);
	if(ret)
	{
		DWC_ERROR("Failed to register IRQ (%#x).\n", ret);
		return ret;
	}
	else
		_core->irq = _irq;

	return 0;
}

/**
 * dwc_otg_core_destroy
 *
 * This shuts down the hardware and releases any resources
 * obtained whilst the driver was active.
 *
 * This function will not free _core.
 */
void dwc_otg_core_destroy(dwc_otg_core_t *_core)
{
	DWC_VERBOSE("%s(%p)\n", __func__, _core);

	if(!_core)
	{
		DWC_ERROR("%s called with null core pointer.\n", __func__);
		return;
	}

	// Shutdown hardware.
	dwc_otg_hw_power(_core, 0);

	// Release IRQ
	if(_core->irq)
		free_irq(_core->irq, _core);
}

/**
 * Helper function for soft_reset, waits for a certain register
 * to have a certain value.
 */
inline int helper_wait_for_reg(volatile uint32_t *_reg, uint32_t _mask, uint32_t _val)
{
	uint32_t count = 0;
	uint32_t curr = dwc_otg_read_reg32(_reg);

	if(_mask == 0)
		return -EINVAL;

	while((curr & _mask) != _val)
	{
		count++;
		if(count > 1000)
		{
			DWC_ERROR("Waited 10 seconds in dwc_otg_core_soft_reset. Bailing.");
			return -EIO;
		}

		msleep(10);
		curr = dwc_otg_read_reg32(_reg);
	}

	return 0;
}

/**
 * dwc_otg_core_soft_reset
 *
 * Soft Resets the core.
 */
int dwc_otg_core_soft_reset(dwc_otg_core_t *_core)
{
	grstctl_data_t grstctl = { .d32 = 0 };
	int ret = 0;

	DWC_VERBOSE("%s(%p)\n", __func__, _core);

	if(!_core)
	{
		DWC_ERROR("%s called with null core pointer.\n", __func__);
		return -EINVAL;
	}

	// Wait for AHBIDLE
	grstctl.b.ahbidle = 1;
	ret = helper_wait_for_reg(&_core->registers->grstctl, grstctl.d32, grstctl.d32);

	if(ret)
		return ret;

	grstctl.b.ahbidle = 0;

	// Set core reset flag
	grstctl.b.csftrst = 1;
	dwc_otg_modify_reg32(&_core->registers->grstctl, grstctl.d32, 0);
	ret = helper_wait_for_reg(&_core->registers->grstctl, grstctl.d32, 0);
	if(ret)
		return ret;

	grstctl.b.csftrst = 0;

	// Wait for AHBIDLE
	grstctl.b.ahbidle = 1;
	ret = helper_wait_for_reg(&_core->registers->grstctl, ~grstctl.d32, 0);
	if(ret)
		return ret;

	msleep(1);

	return 0;
}

/**
 * dwc_otg_core_start
 *
 * Start chip operation.
 */
int dwc_otg_core_start(dwc_otg_core_t *_core)
{
	int i = 0;
	gahbcfg_data_t gahbcfg = { .d32 = 0 };
	gusbcfg_data_t gusbcfg = { .d32 = 0 };
	dcfg_data_t dcfg = { .d32 = 0 };
	dctl_data_t dctl = { .d32 = 0 };
	gotgctl_data_t gotgctl = { .d32 = 0 };
	//depctl_data_t depctl = { .d32 = 0 };
	gintmsk_data_t gintmsk = { .d32 = 0 };
	//diepmsk_data_t diepmsk = { .d32 = 0 };
	//doepmsk_data_t doepmsk = { .d32 = 0 };

	DWC_VERBOSE("%s(%p)\n", __func__, _core);

	// Set some configuration
	gahbcfg.b.glblintrmsk = 1; // Enable Interrupts
	gahbcfg.b.dmaenable = DWC_GAHBCFG_DMAENABLE;
	gahbcfg.b.hburstlen = DWC_GAHBCFG_INT_DMA_BURST_INCR8;
	dwc_otg_write_reg32(&_core->registers->gahbcfg, gahbcfg.d32);

	// Set USB Configuration
	gusbcfg.b.phyif = 1;
	gusbcfg.b.srpcap = 1;
	gusbcfg.b.hnpcap = 1;
	gusbcfg.b.usbtrdtim = 5;
	dwc_otg_write_reg32(&_core->registers->gusbcfg, gusbcfg.d32);

	// Set Device Configuration
	dcfg.b.devspd = DWC_DCFG_HIGH_SPEED;
	dcfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;
	dwc_otg_write_reg32(&_core->device_registers->dcfg, dcfg.d32);

	// Enable EP0
	//depctl.b.epena = 1;
	//dwc_otg_write_reg32(&_core->in_ep_registers[0]->diepctl, depctl.d32);
	//dwc_otg_write_reg32(&_core->out_ep_registers[0]->doepctl, depctl.d32);

	// Write FIFO sizes
	dwc_otg_write_reg32(&_core->registers->grxfsiz, DWC_OTG_RX_FIFO_SIZE);
	dwc_otg_write_reg32(&_core->registers->gnptxfsiz, (DWC_OTG_TX_FIFO_OFFSET << 16) | DWC_OTG_TX_FIFO_SIZE);

	// Clear Interrupts
	for(i = 0; i < _core->num_eps; i++)
	{
		dwc_otg_write_reg32(&_core->in_ep_registers[i]->diepint, 0xffffffff);
		dwc_otg_write_reg32(&_core->out_ep_registers[i]->doepint, 0xffffffff);
	}

	// We're ready!
	_core->ready = 1;

	// Enable Interrupts
	gintmsk.b.otgintr = 1;
	gintmsk.b.usbsuspend = 1;
	gintmsk.b.usbreset = 1;
	gintmsk.b.inepintr = 1;
	gintmsk.b.outepintr = 1;
	gintmsk.b.disconnect = 1;
	dwc_otg_write_reg32(&_core->registers->gintmsk, gintmsk.d32);
	dwc_otg_write_reg32(&_core->device_registers->daintmsk, 0); //0xffffffff);

	// Enable EP Interrupts
	/*diepmsk.b.xfercompl = 1;
	diepmsk.b.ahberr = 1;
	diepmsk.b.timeout = 1;
	dwc_otg_write_reg32(&_core->device_registers->diepmsk, diepmsk.d32);

	doepmsk.b.xfercompl = 1;
	doepmsk.b.setup = 1;
	doepmsk.b.back2backsetup = 1;
	dwc_otg_write_reg32(&_core->device_registers->doepmsk, doepmsk.d32);*/

	// Clear EP0 interrupts
	//dwc_otg_write_reg32(&_core->in_ep_registers[0]->diepint, 0xffffffff);
	//dwc_otg_write_reg32(&_core->out_ep_registers[0]->doepint, 0xffffffff);

	dctl.b.pwronprgdone = 1;
	dctl.b.cgoutnak = 1;
	dctl.b.cgnpinnak = 1;
	dwc_otg_write_reg32(&_core->device_registers->dctl, dctl.d32);

	gotgctl.b.sesreq = 1;
	dwc_otg_modify_reg32(&_core->registers->gotgctl, 0, gotgctl.d32);

	return 0;
}

/**
 * dwc_otg_core_stop
 *
 * Stop chip operation.
 */
void dwc_otg_core_stop(dwc_otg_core_t *_core)
{
	DWC_VERBOSE("%s(%p)\n", __func__, _core);
}

/**
 * dwc_otg_core_ep_reset
 *
 * Resets all of the endpoints, cancelling any
 * current requests.
 */
void dwc_otg_core_ep_reset(dwc_otg_core_t *_core)
{
	uint32_t i;

	_core->ready = 0;

	// Cancel all current requests.
	dwc_otg_core_cancel_all_requests(_core);

	// Set NAK on all out endpoints
	for(i = 0; i < _core->num_eps; i++)
	{
		depctl_data_t depctl = { .d32 = 0 };

		_core->endpoints[i].active = 0;
		
		depctl.b.snak = 1;
		dwc_otg_modify_reg32(&_core->out_ep_registers[i]->doepctl, 0, depctl.d32);
	}

	_core->ready = 1;
}

/**
 * dwc_otg_core_enable_interrupts
 *
 * Enables interrupts.
 */
int dwc_otg_core_enable_interrupts(dwc_otg_core_t *_core)
{
	gahbcfg_data_t gahbcfg = { .d32 = 0 };

	DWC_VERBOSE("%s(%p)\n", __func__, _core);

	gahbcfg.b.glblintrmsk = 1;
	dwc_otg_modify_reg32(&_core->registers->gahbcfg, 0, gahbcfg.d32);

	return 0;
}

/**
 * dwc_otg_core_disable_interrupts
 *
 * Disables interrupts.
 */
int dwc_otg_core_disable_interrupts(dwc_otg_core_t *_core)
{
	gahbcfg_data_t gahbcfg = { .d32 = 0 };

	DWC_VERBOSE("%s(%p)\n", __func__, _core);

	gahbcfg.b.glblintrmsk = 1;
	dwc_otg_modify_reg32(&_core->registers->gahbcfg, gahbcfg.d32, 0);

	return 0;
}

/**
 * The core USB IRQ handler.
 */
irqreturn_t dwc_otg_core_irq(int _irq, void *_dev)
{
	dwc_otg_core_t *core = (dwc_otg_core_t*)_dev;
	gintsts_data_t gintsts = { .d32 = 0 };
	gintsts_data_t gintclr = { .d32 = 0 };

	gintsts.d32 = dwc_otg_read_reg32(&core->registers->gintsts) & dwc_otg_read_reg32(&core->registers->gintmsk);

	DWC_VERBOSE("%s(%d, %p) gintsts=0x%08x\n", __func__, _irq, _dev, gintsts.d32);

	if(gintsts.b.otgintr)
	{
		DWC_DEBUG("otgintr\n");

		// Clear OTG interrupts
		dwc_otg_write_reg32(&core->registers->gotgint, 0xffffffff);

		gintclr.b.otgintr = 1;
	}

	if(gintsts.b.usbsuspend)
	{
		DWC_DEBUG("usbsuspend\n");
		gintclr.b.usbsuspend = 1;
	}

	if(gintsts.b.disconnect)
	{
		DWC_DEBUG("disconnect\n");
		gintclr.b.disconnect = 1;
	}

	// Clear the interrupts
	dwc_otg_write_reg32(&core->registers->gintsts, gintclr.d32);
	return gintclr.d32 == 0 ? IRQ_NONE : IRQ_HANDLED;
}

/**
 * Enable an endpoint.
 */
int dwc_otg_core_enable_ep(dwc_otg_core_t *_core, dwc_otg_core_ep_t *_ep, struct usb_endpoint_descriptor *_desc)
{
	_ep->descriptor = _desc;
	return 0;
}

/**
 * Disable an endpoint.
 */
int dwc_otg_core_disable_ep(dwc_otg_core_t *_core, dwc_otg_core_ep_t *_ep)
{
	return 0;
}

/**
 * Stall an endpoint.
 */
int dwc_otg_core_stall_ep(dwc_otg_core_t *_core, dwc_otg_core_ep_t *_ep)
{
	// TODO: fix this shit.
	return 0;
}

/**
 * dwc_otg_core_enqueue_request
 *
 * Queue an out request.
 */
int dwc_otg_core_enqueue_request(dwc_otg_core_t *_core, dwc_otg_core_ep_t *_ep, dwc_otg_core_request_t *_req)
{
	// If the list is empty, we will need to start the EP up.
	// Store this now, as we're about to modify the list.
	uint32_t startEP = list_empty(&_ep->transfer_queue);

	DWC_VERBOSE("%s(%p, %p, %p)\n", __func__, _core, _ep, _req);

	if(_req->queued == 1)
	{
		DWC_ERROR("Tried to queue a request twice!\n");
		return -EINVAL;
	}

	if(_req->length && !_req->buffer)
	{
		DWC_ERROR("Tried to send a request with no buffer, yet a length.\n");
		return -EINVAL;
	}
	else if(_req->length)
	{
		if(_req->dont_free && !_req->dma_buffer)
		{
			DWC_WARNING("Allocating a DMA buffer for a request that won't be freed.\n");
		}

		if(!_req->dont_free && _req->dma_buffer)
		{
			DWC_WARNING("Pre-allocated DMA buffer in a request that's going to be freed.\n");
		}
	}

	_req->core = _core;
	_req->ep = _ep;
	_req->queued = 1;

	if(_req->length && !_req->dma_buffer)
	{
		// Allocate DMA buffer
		_req->dma_buffer = dma_alloc_coherent(NULL, _req->length, &_req->dma_address, GFP_KERNEL);
		if(!_req->dma_buffer)
		{
			DWC_ERROR("Failed to allocate a buffer for DMA.\n");
			return -EIO;
		}

		memcpy(_req->dma_buffer, _req->buffer, sizeof(_req->length));
	}

	// TODO: Do a lock here

	INIT_LIST_HEAD(&_req->queue_pointer);
	list_add_tail(&_req->queue_pointer, &_ep->transfer_queue);
	
	if(startEP)
		dwc_otg_core_start_request(_core, _req);

	// TODO: End lock
	
	return 0;
}

/**
 * dwc_otg_core_start_request
 */
int dwc_otg_core_start_request(dwc_otg_core_t *_core, dwc_otg_core_request_t *_req)
{
	dwc_otg_core_ep_t *ep = _req->ep;
	depctl_data_t depctl = { .d32 = 0 };
	daint_data_t daint = { .d32 = 0 };
	volatile uint32_t *depctl_ptr;
	volatile uint32_t *depdma_ptr;
	volatile uint32_t *deptsiz_ptr;
	unsigned long flags;

	DWC_VERBOSE("%s(%p, %p)\n", __func__, _core, _req);

	// Reset Flags
	_req->setup = 0;
	_req->cancelled = 0;
	_req->active = 1;

	spin_lock_irqsave(&ep->lock, flags);

	if(_req->direction == DWC_OTG_REQUEST_OUT)
	{
		daint.ep.out = 1 << ep->num;
		depctl_ptr = &ep->out_registers->doepctl;
		depdma_ptr = &ep->out_registers->doepdma;
		deptsiz_ptr = &ep->out_registers->doeptsiz;
	}
	else
	{
		daint.ep.in = 1 << ep->num;
		depctl_ptr = &ep->in_registers->diepctl;
		depdma_ptr = &ep->in_registers->diepdma;
		deptsiz_ptr = &ep->in_registers->dieptsiz;
	}

	dwc_otg_modify_reg32(&_core->device_registers->daintmsk, 0, daint.d32);
	dwc_otg_write_reg32(depdma_ptr, _req->dma_address);
	depctl.d32 = dwc_otg_read_reg32(depctl_ptr);

	/*if(depctl.b.epena)
	{
		if(depctl.b.dpid)
			depctl.b.setd0pid = 1;
		else
			depctl.b.setd1pid = 1;
	}*/

	if(!ep->active)
	{
		INIT_LIST_HEAD(&ep->queue_pointer);
		list_add_tail(&ep->queue_pointer, &_core->ep_transfer_queue);
		ep->active = 1;
	}

	depctl.b.epena = 1;
	depctl.b.cnak = 1;
	depctl.b.usbactep = 1;

	if(ep->num == 0)
	{
		deptsiz0_data_t deptsiz0 = { .d32 = 0 };
		deptsiz0.b.pktcnt = 1;
		deptsiz0.b.supcnt = 1;

		if(_req->length &~ 0x7f)
		{
			DWC_WARNING("Packet too large for EP0 (0x%0x).\n", _req->length);
			deptsiz0.b.xfersize = 0x7f;
		}
		else
			deptsiz0.b.xfersize = _req->length;

		dwc_otg_write_reg32(deptsiz_ptr, deptsiz0.d32);

		//depctl.b.mps = 64; // EP0 only supports an MPS of 64.
		dwc_otg_write_reg32(depctl_ptr, depctl.d32);
	}
	else
	{
		deptsiz_data_t deptsiz = { .d32 = 0 };
		int mps = dwc_otg_mps_from_speed(ep->speed);

		deptsiz.b.pktcnt = _req->length == 0 ? 1 : (_req->length + mps - 1)/mps;
		deptsiz.b.xfersize = _req->length;
		dwc_otg_write_reg32(deptsiz_ptr, deptsiz.d32);

		depctl.b.eptype = _req->request_type;
		depctl.b.mps = mps;
		dwc_otg_write_reg32(depctl_ptr, depctl.d32);
	}

	/*if(!depctl.b.usbactep)
	{
		struct list_head *list_ptr = _core->ep_transfer_queue.prev->prev;
		dwc_otg_core_ep_t *ep2 = list_entry(list_ptr, dwc_otg_core_ep_t, queue_pointer);
		depctl_data_t depctl = { .d32 = 0 };

		// TODO: Set nextep. : P
		depctl.d32 = dwc_otg_read_reg32(&ep2->in_registers->diepctl);
		depctl.b.nextep = ep->num;
		dwc_otg_write_reg32(&ep2->in_registers->diepctl, depctl.d32);
	}*/

	spin_unlock_irqrestore(&ep->lock, flags);

	return 0;
}

/**
 * dwc_otg_core_complete_request
 */
int dwc_otg_core_complete_request(dwc_otg_core_t *_core, dwc_otg_core_request_t *_req)
{
	dwc_otg_core_ep_t *ep = _req->ep;
	dwc_otg_core_request_t *req = list_first_entry(&ep->transfer_queue, dwc_otg_core_request_t, queue_pointer);
	deptsiz_data_t deptsiz = { .d32 = 0 };
	volatile uint32_t *depctl_ptr;
	volatile uint32_t *deptsiz_ptr;

	DWC_VERBOSE("%s(%p, %p)\n", __func__, _core, _req);

	if(req != _req)
	{
		DWC_ERROR("Tried to complete request out of phase!\n");
		return -EIO;
	}

	// Setup register pointers
	if(_req->direction == DWC_OTG_REQUEST_OUT)
	{
		depctl_ptr = &ep->out_registers->doepctl;
		deptsiz_ptr = &ep->out_registers->doeptsiz;
	}
	else
	{
		depctl_ptr = &ep->in_registers->diepctl;
		deptsiz_ptr = &ep->in_registers->dieptsiz;
	}

	// Remove this request as it is now complete.
	list_del(&_req->queue_pointer);

	// Remove us from the EP queue.
	if(list_empty(&ep->transfer_queue))
	{
		list_del(&ep->queue_pointer);
		ep->active = 0;
	}

	// Calculate how much was actually sent
	deptsiz.d32 = dwc_otg_read_reg32(deptsiz_ptr);
	req->length -= deptsiz.b.xfersize;

	// If there are more requests to process on this EP...
	if(!list_empty(&ep->transfer_queue) && _core->ready)
	{
		// Start the next one!
		req = list_first_entry(&ep->transfer_queue, dwc_otg_core_request_t, queue_pointer);
		dwc_otg_core_start_request(_core, req);
	}
	/*else
	{
		depctl_data_t depctl = { .d32 = 0 };

		// Shutdown the EP.
		depctl.b.snak = 1;
		depctl.b.epdis = 1;
		dwc_otg_write_reg32(depctl_ptr, depctl.d32);
	}*/

	// Mark the request as inactive
	_req->active = 0;
	_req->queued = 0;

	// Call the handler
	DWC_VERBOSE("%s: calling handler. cancel=%p, compl=%p\n", __func__, _req->cancelled_handler, req->completed_handler);
	if(_req->cancelled && _req->cancelled_handler)
		_req->cancelled_handler(_req);
	else if(_req->completed_handler)
		_req->completed_handler(_req);
	
	if(!_req->dont_free)
	{
		if(_req->dma_buffer)
			dma_free_coherent(NULL, _req->length, _req->dma_buffer, _req->dma_address);
		kfree(_req->buffer);
		kfree(_req);
		_req = NULL;
	}

	return 0;
}

/**
 * Cancel all current requests.
 */
int dwc_otg_core_cancel_all_requests(dwc_otg_core_t *_core)
{
	while(!list_empty(&_core->ep_transfer_queue))
	{
		dwc_otg_core_ep_t *ep = list_first_entry(&_core->ep_transfer_queue, dwc_otg_core_ep_t, queue_pointer);

		DWC_VERBOSE("Deleting transfers from %s (%p).\n", ep->name, ep);

		if(!ep->active)
		{
			DWC_ERROR("Inactive EP (%s) made it onto the transfer queue!\n", ep->name);
			list_del(&ep->queue_pointer);
			continue;
		}

		if(list_empty(&ep->transfer_queue))
		{
			DWC_ERROR("%s on transfer queue, yet has no transfers queued!\n", ep->name);
			list_del(&ep->queue_pointer);
			ep->active = 0;
			continue;
		}

		do
		{
			dwc_otg_core_request_t *req = list_first_entry(&ep->transfer_queue, dwc_otg_core_request_t, queue_pointer);

			DWC_VERBOSE("Completing request %p, on %s (%p).\n", req, ep->name, ep);

			// Cancel the request.
			req->cancelled = 1;
			dwc_otg_core_complete_request(_core, req);
		}
 		while(!list_empty(&ep->transfer_queue));
	}

	return 0;
}
