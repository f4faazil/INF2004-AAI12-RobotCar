#ifndef TEST_MOCK_TKERNEL_H
#define TEST_MOCK_TKERNEL_H

#include <stdint.h>

typedef int32_t ID;
typedef int32_t ER;
typedef int32_t TMO;
typedef struct { uint32_t mtxatr; } T_CMTX;

#define E_OK (0)
#define E_TMOUT (-50)
#define TMO_POL (0)
#define TA_TFIFO (0U)
#define TA_INHERIT (2U)

ID tk_cre_mtx(T_CMTX *pk_cmtx);
ER tk_del_mtx(ID mtxid);
ER tk_loc_mtx(ID mtxid, TMO tmout);
ER tk_unl_mtx(ID mtxid);

#endif
