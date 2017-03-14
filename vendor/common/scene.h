#pragma once

#include "../../proj/tl_common.h"

#define __CT__  0

typedef struct{
	u8 id;
	u8 lum;
	u8 rgb[3];
#if __CT__
	u8 ct;   //0-100%
	u8 rsv[2];
#else
    u8 rsv[3];
#endif
}scene_t;

enum{
    SCENE_DEL = 0,
    SCENE_ADD = 1,
    SCENE_MAX,
};
#define SCENE_MESH_NOTIFY_CNT         6
#define MAX_SCENE_CNT   16
extern scene_t scene_data[];

extern void scene_init(void);
extern void scene_flash_clean(void);
extern u8 scene_find(u8 id);
extern u8 scene_add(scene_t * data);
extern u8 scene_del(u8 id);
extern u8 scene_load(u8 id);

void scene_handle();
int is_scene_poll_notify_busy();
void scene_poll_notify_init(u8 need_bridge);
int scene_poll_notify(u8 *pkt_rsp);
void scene_rsp_mesh();
int scene_get_by_id(u8 *pkt_rsp, u8 id);
int scene_get_all_id(u8 *pkt_rsp);

