/*
 *  Copyright (c) 2000-2008 LSI Corporation.
 *
 *
 *           Name:  mpi2_tool.h
 *          Title:  MPI diagnostic tool structures and definitions
 *  Creation Date:  March 26, 2007
 *
 *    mpi2_tool.h Version:  02.00.02
 *
 *  Version History
 *  ---------------
 *
 *  Date      Version   Description
 *  --------  --------  ------------------------------------------------------
 *  04-30-07  02.00.00  Corresponds to Fusion-MPT MPI Specification Rev A.
 *  12-18-07  02.00.01  Added Diagnostic Buffer Post and Diagnostic Release
 *                      structures and defines.
 *  02-29-08  02.00.02  Modified various names to make them 32-character unique.
 *  --------------------------------------------------------------------------
 */

#ifndef MPI2_TOOL_H
#define MPI2_TOOL_H

/*****************************************************************************
*
*               Toolbox Messages
*
*****************************************************************************/

/* defines for the Tools */
#define MPI2_TOOLBOX_CLEAN_TOOL                     (0x00)
#define MPI2_TOOLBOX_MEMORY_MOVE_TOOL               (0x01)
#define MPI2_TOOLBOX_BEACON_TOOL                    (0x05)

/****************************************************************************
*  Toolbox reply
****************************************************************************/

typedef struct _MPI2_TOOLBOX_REPLY
{
    U8                      Tool;                       /* 0x00 */
    U8                      Reserved1;                  /* 0x01 */
    U8                      MsgLength;                  /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U16                     Reserved5;                  /* 0x0C */
    U16                     IOCStatus;                  /* 0x0E */
    U32                     IOCLogInfo;                 /* 0x10 */
} MPI2_TOOLBOX_REPLY, MPI2_POINTER PTR_MPI2_TOOLBOX_REPLY,
  Mpi2ToolboxReply_t, MPI2_POINTER pMpi2ToolboxReply_t;


/****************************************************************************
*  Toolbox Clean Tool request
****************************************************************************/

typedef struct _MPI2_TOOLBOX_CLEAN_REQUEST
{
    U8                      Tool;                       /* 0x00 */
    U8                      Reserved1;                  /* 0x01 */
    U8                      ChainOffset;                /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U32                     Flags;                      /* 0x0C */
   } MPI2_TOOLBOX_CLEAN_REQUEST, MPI2_POINTER PTR_MPI2_TOOLBOX_CLEAN_REQUEST,
  Mpi2ToolboxCleanRequest_t, MPI2_POINTER pMpi2ToolboxCleanRequest_t;

/* values for the Flags field */
#define MPI2_TOOLBOX_CLEAN_BOOT_SERVICES            (0x80000000)
#define MPI2_TOOLBOX_CLEAN_PERSIST_MANUFACT_PAGES   (0x40000000)
#define MPI2_TOOLBOX_CLEAN_OTHER_PERSIST_PAGES      (0x20000000)
#define MPI2_TOOLBOX_CLEAN_FW_CURRENT               (0x10000000)
#define MPI2_TOOLBOX_CLEAN_FW_BACKUP                (0x08000000)
#define MPI2_TOOLBOX_CLEAN_MEGARAID                 (0x02000000)
#define MPI2_TOOLBOX_CLEAN_INITIALIZATION           (0x01000000)
#define MPI2_TOOLBOX_CLEAN_FLASH                    (0x00000004)
#define MPI2_TOOLBOX_CLEAN_SEEPROM                  (0x00000002)
#define MPI2_TOOLBOX_CLEAN_NVSRAM                   (0x00000001)


/****************************************************************************
*  Toolbox Memory Move request
****************************************************************************/

typedef struct _MPI2_TOOLBOX_MEM_MOVE_REQUEST
{
    U8                      Tool;                       /* 0x00 */
    U8                      Reserved1;                  /* 0x01 */
    U8                      ChainOffset;                /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    MPI2_SGE_SIMPLE_UNION   SGL;                        /* 0x0C */
} MPI2_TOOLBOX_MEM_MOVE_REQUEST, MPI2_POINTER PTR_MPI2_TOOLBOX_MEM_MOVE_REQUEST,
  Mpi2ToolboxMemMoveRequest_t, MPI2_POINTER pMpi2ToolboxMemMoveRequest_t;


/****************************************************************************
*  Toolbox Beacon Tool request
****************************************************************************/

typedef struct _MPI2_TOOLBOX_BEACON_REQUEST
{
    U8                      Tool;                       /* 0x00 */
    U8                      Reserved1;                  /* 0x01 */
    U8                      ChainOffset;                /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U8                      Reserved5;                  /* 0x0C */
    U8                      PhysicalPort;               /* 0x0D */
    U8                      Reserved6;                  /* 0x0E */
    U8                      Flags;                      /* 0x0F */
} MPI2_TOOLBOX_BEACON_REQUEST, MPI2_POINTER PTR_MPI2_TOOLBOX_BEACON_REQUEST,
  Mpi2ToolboxBeaconRequest_t, MPI2_POINTER pMpi2ToolboxBeaconRequest_t;

/* values for the Flags field */
#define MPI2_TOOLBOX_FLAGS_BEACONMODE_OFF       (0x00)
#define MPI2_TOOLBOX_FLAGS_BEACONMODE_ON        (0x01)


/*****************************************************************************
*
*       Diagnostic Buffer Messages
*
*****************************************************************************/


/****************************************************************************
*  Diagnostic Buffer Post request
****************************************************************************/

typedef struct _MPI2_DIAG_BUFFER_POST_REQUEST
{
    U8                      Reserved1;                  /* 0x00 */
    U8                      BufferType;                 /* 0x01 */
    U8                      ChainOffset;                /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U64                     BufferAddress;              /* 0x0C */
    U32                     BufferLength;               /* 0x14 */
    U32                     Reserved5;                  /* 0x18 */
    U32                     Reserved6;                  /* 0x1C */
    U32                     Flags;                      /* 0x20 */
    U32                     ProductSpecific[23];        /* 0x24 */
} MPI2_DIAG_BUFFER_POST_REQUEST, MPI2_POINTER PTR_MPI2_DIAG_BUFFER_POST_REQUEST,
  Mpi2DiagBufferPostRequest_t, MPI2_POINTER pMpi2DiagBufferPostRequest_t;

/* values for the BufferType field */
#define MPI2_DIAG_BUF_TYPE_TRACE                    (0x00)
#define MPI2_DIAG_BUF_TYPE_SNAPSHOT                 (0x01)
/* count of the number of buffer types */
#define MPI2_DIAG_BUF_TYPE_COUNT                    (0x02)


/****************************************************************************
*  Diagnostic Buffer Post reply
****************************************************************************/

typedef struct _MPI2_DIAG_BUFFER_POST_REPLY
{
    U8                      Reserved1;                  /* 0x00 */
    U8                      BufferType;                 /* 0x01 */
    U8                      MsgLength;                  /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U16                     Reserved5;                  /* 0x0C */
    U16                     IOCStatus;                  /* 0x0E */
    U32                     IOCLogInfo;                 /* 0x10 */
    U32                     TransferLength;             /* 0x14 */
} MPI2_DIAG_BUFFER_POST_REPLY, MPI2_POINTER PTR_MPI2_DIAG_BUFFER_POST_REPLY,
  Mpi2DiagBufferPostReply_t, MPI2_POINTER pMpi2DiagBufferPostReply_t;


/****************************************************************************
*  Diagnostic Release request
****************************************************************************/

typedef struct _MPI2_DIAG_RELEASE_REQUEST
{
    U8                      Reserved1;                  /* 0x00 */
    U8                      BufferType;                 /* 0x01 */
    U8                      ChainOffset;                /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
} MPI2_DIAG_RELEASE_REQUEST, MPI2_POINTER PTR_MPI2_DIAG_RELEASE_REQUEST,
  Mpi2DiagReleaseRequest_t, MPI2_POINTER pMpi2DiagReleaseRequest_t;


/****************************************************************************
*  Diagnostic Buffer Post reply
****************************************************************************/

typedef struct _MPI2_DIAG_RELEASE_REPLY
{
    U8                      Reserved1;                  /* 0x00 */
    U8                      BufferType;                 /* 0x01 */
    U8                      MsgLength;                  /* 0x02 */
    U8                      Function;                   /* 0x03 */
    U16                     Reserved2;                  /* 0x04 */
    U8                      Reserved3;                  /* 0x06 */
    U8                      MsgFlags;                   /* 0x07 */
    U8                      VP_ID;                      /* 0x08 */
    U8                      VF_ID;                      /* 0x09 */
    U16                     Reserved4;                  /* 0x0A */
    U16                     Reserved5;                  /* 0x0C */
    U16                     IOCStatus;                  /* 0x0E */
    U32                     IOCLogInfo;                 /* 0x10 */
} MPI2_DIAG_RELEASE_REPLY, MPI2_POINTER PTR_MPI2_DIAG_RELEASE_REPLY,
  Mpi2DiagReleaseReply_t, MPI2_POINTER pMpi2DiagReleaseReply_t;


#endif
