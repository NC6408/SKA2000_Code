#include <stdio.h>
#include <stdlib.h>

#include "SKA_Logger.h"

// 日志分类
zlog_category_t *pCategory = NULL;

void SKA_Logger_Create() {
    int rc = zlog_init("./conf/zlog.conf");
    if (rc != 0) {
        fprintf(stderr, "init zlog file './conf/zlog.conf' failed: %d\n", rc);
        exit(EXIT_FAILURE);
    }
    pCategory = zlog_get_category("antisway_stdout");
    if (pCategory == NULL) {
        fprintf(stderr, "get category 'antisway_stdout' failed\n");
        zlog_fini();
        exit(EXIT_FAILURE);
    }
}

void SKA_Logger_Destroy() {
    zlog_fini();
}