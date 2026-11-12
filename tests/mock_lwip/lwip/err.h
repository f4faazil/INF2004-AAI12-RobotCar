#ifndef MOCK_LWIP_ERR
#define MOCK_LWIP_ERR
typedef int err_t;
#define ERR_OK 0
#define ERR_MEM (-1)
#define ERR_BUF (-2)
#define ERR_WOULDBLOCK (-7)
#define ERR_CONN (-11)
#define ERR_ARG (-16)
#define ERR_ABRT (-13)
#define ERR_RST (-14)
#endif
