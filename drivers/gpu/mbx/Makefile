obj-$(CONFIG_IPHONE_MBX) += mbxaccess.o omaplcd.o 

mbxaccess-objs	:= mbxaccess_main.o malloc_debug.o mbxaccess_iowrapper.o mbxaccess_mbxmem.o mbxaccess_memwrapper.o \
	mbxaccess_mmapwrapper.o mbxaccess_procwrapper.o mbxaccess_pvrclassjtable.o mbxaccess_timerwrapper.o \
	mbxaccess_virtmemwrapper.o mmap.o mm.o mutex.o pvr_debug.o

omaplcd-objs := omaplcd_displayclass.o omaplcd_linux.o

all:
	make -C $(KERNELDIR) M=$(PWD) modules
clean:
	make -C $(KERNELDIR) M=$(PWD) clean
