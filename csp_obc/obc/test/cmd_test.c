/*
 * cmd_test.c
 *
 *  Created on: 2016年7月27日
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "command.h"

#include "bsp_ad7490.h"
#include "obc_mem.h"
#include "contrl.h"
#include "ctrl_cmd_types.h"
#include "driver_debug.h"
#include "bsp_cpu_flash.h"
#include "crc.h"
//#include "bsp_camera.h"
#include "switches.h"
#include "hk.h"
#include "bsp_pca9665.h"

#include "FreeRTOS.h"
#include "task.h"
#include "route.h"
#include "router_io.h"
#include "dtb_805.h"
#include "camera_805.h"

//static uint32_t my_test_ccrram __attribute__((section(".ram_persist"))) = 5;

extern char __ram_persist_start;
extern char __ram_persist_end;
extern char __vectors_start;
extern char __exidx_start;
extern char __exidx_end;
extern char __etext;
extern char __data_start__;
extern char __data_end__;
extern char __bss_start__;
extern char __bss_end__;
extern char _noinit;
extern char _end_noinit;
extern char __heap_start__;


//static uint8_t array_myfiles[64*1024] = {0};
//
//int save_files(struct command_context *ctx __attribute__((unused))){
//
//	bsp_EraseCpuFlash(ADDR_FLASH_SECTOR_8);
//
//	cup_flash_content_t * pointer = (cup_flash_content_t *)ObcMemMalloc(sizeof(cup_flash_content_t));
//
//	pointer->index = 0;
//	pointer->len = 91;
//	uint32_t j = 0;
//	uint16_t length = 100;
//	while(length --)
//	{
//		pointer->id = 0;
//		pointer->info = ((pointer->index << 12) | (pointer->id));
//		for (uint8_t i = 0; i < 91; i++)
//		{
//			pointer->data[i] = array_myfiles[j];
//			j ++;
//		}
//		pointer->id ++;
//		SavePermanentAudioFiles(pointer);
//	}
//	return CMD_ERROR_NONE;
//}



int write_flansh(struct command_context *ctx ){

	uint32_t index, id;
	uint32_t  len;

	char * args = command_args(ctx);

	cup_flash_content_t * pointer = NULL;
	pointer = (cup_flash_content_t *)ObcMemMalloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
		return CMD_ERROR_SYNTAX;

	pointer->id = id;
	pointer->len = len;
	pointer->index = index;

	for(uint8_t i=0;i<(pointer->len);i++)
		pointer->data[i] = i;
	SavePermanentAudioFiles(pointer);

	ObcMemFree(pointer);

	return CMD_ERROR_NONE;
}


int continuously_write_Permanent_flansh(struct command_context *ctx ){

	uint32_t index, id, len, framesize;
	cup_flash_content_t * pointer = NULL;
	char * args = command_args(ctx);

	pointer = (cup_flash_content_t *)ObcMemMalloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u %u", &index, &id, &len, &framesize) != 4)
		return CMD_ERROR_SYNTAX;
	if((index < 0) || (index > 3)){
		driver_debug(DEBUG_FLASH, "index input error\r\n");
		return 1;
	}
	if((id < 0) || (id > 345)){
		driver_debug(DEBUG_FLASH, "id input error\r\n");
		return 2;
	}
	if((len < 0) || (len > 91)){
		driver_debug(DEBUG_FLASH, "len input error\r\n");
		return 3;
	}
	if(((id + framesize) < 0) || ((id + framesize) > 345)){
		driver_debug(DEBUG_FLASH, "framesize input error\r\n");
		return 4;
	}


	for(uint16_t i = 0;i < framesize;i ++)
	{
		pointer->info= ((index << 12) | id);
//		pointer->index = index;
//		pointer->id = id;
		id ++;
		pointer->len = len;

		for(uint16_t i=0;i<(pointer->len);i++)
			pointer->data[i] = i;
		SavePermanentAudioFiles(pointer);
	}

	ObcMemFree(pointer);
	return CMD_ERROR_NONE;
}



int continuously_write_new_flansh(struct command_context *ctx ){

	uint32_t index, id, len, framesize;
	cup_flash_content_t * pointer = NULL;
	char * args = command_args(ctx);

	pointer = (cup_flash_content_t *)ObcMemMalloc(sizeof(cup_flash_content_t));
	if(sscanf(args, "%u %u %u %u", &index, &id, &len, &framesize) != 4)
		return CMD_ERROR_SYNTAX;
	if((index < 0) || (index > 3)){
		driver_debug(DEBUG_FLASH, "index input error\r\n");
		return 1;
	}
	if((id < 0) || (id > 345)){
		driver_debug(DEBUG_FLASH, "id input error\r\n");
		return 2;
	}
	if((len < 0) || (len > 91)){
		driver_debug(DEBUG_FLASH, "len input error\r\n");
		return 3;
	}
	if(((id + framesize) < 0) || ((id + framesize) > 345)){
		driver_debug(DEBUG_FLASH, "framesize input error\r\n");
		return 4;
	}


	for(uint16_t i = 0;i < framesize;i ++)
	{
		pointer->info= ((index << 12) | id);
		id++;
		pointer->len = len;

		for(uint16_t i=0;i<(pointer->len);i++)
			pointer->data[i] = i;
		SaveNewAudioFiles(pointer);
	}

	ObcMemFree(pointer);
	return CMD_ERROR_NONE;
}



int read_Permanent_flansh(struct command_context *ctx )
{
	char * args = command_args(ctx);
	uint32_t index, id, len;
	cup_flash_cmd_t * pointer = NULL;

	pointer = (cup_flash_cmd_t *)ObcMemMalloc(sizeof(cup_flash_cmd_t));

	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
			return CMD_ERROR_SYNTAX;

	pointer->info = ((index << 12) | id);
//	pointer->index = index;
//	pointer->id = id;
	pointer->len = len;

	xTaskCreate(DownloadSavedAudioFiles, "DownloadSavedAudioFiles", configMINIMAL_STACK_SIZE * 2,
			pointer, tskIDLE_PRIORITY + 4, NULL);

	ObcMemFree(pointer);
	return CMD_ERROR_NONE;

}


int read_new_flansh(struct command_context *ctx )
{
	char * args = command_args(ctx);
	uint32_t index, id, len;
	cup_flash_cmd_t * pointer = NULL;

	pointer = (cup_flash_cmd_t *)ObcMemMalloc(sizeof(cup_flash_cmd_t));

	if(sscanf(args, "%u %u %u", &index, &id, &len) != 3)
			return CMD_ERROR_SYNTAX;

	pointer->info = ((index << 12) | id);
//	pointer->index = index;
//	pointer->id = id;
	pointer->len = len;

	xTaskCreate(DownloadNewAudioFiles, "DownloadNewAudioFiles", configMINIMAL_STACK_SIZE * 2,
			pointer, tskIDLE_PRIORITY + 4, NULL);

	ObcMemFree(pointer);
	return CMD_ERROR_NONE;

}

int I2C_SendToADChip(struct command_context * context __attribute__((unused))){
	uint8_t SendData = 0x10;
	uint8_t ReceiveData[2];
	i2c_master_transaction(0,0x52,&SendData,1,NULL,0,0);
	vTaskDelay(1000);
	if(i2c_master_transaction(0,0x53,NULL,0,ReceiveData,2,20) == -1){
		for(uint8_t i = 0; i < 2; i++){
			driver_debug(DEBUG_I2C,"0x%02x \r\n", ReceiveData[i]);
		}
	}
	return CMD_ERROR_NONE;
}

int crc16_generate(struct command_context * context __attribute__((unused))){
	uint8_t* pdata = ObcMemMalloc(21);
	pdata[0] = 0x42;
	pdata[1] = 0x55;
	pdata[2] = 0x53;
	pdata[3] = 0x41;
	pdata[4] = 0x54;
	pdata[5] = 0x42;
	pdata[6] = 0x4a;
	pdata[7] = 0x42;
	pdata[8] = 0x55;
	pdata[9] = 0x53;
	pdata[10] = 0x41;
	pdata[11] = 0x54;
	pdata[12] = 0x42;
	pdata[13] = 0x4a;
	pdata[14] = 0x03;
	pdata[15] = 0xf0;
	pdata[16] = 0x00;
	pdata[17] = 0x03;
	pdata[18] = 0x05;
	pdata[19] = 0x01;
	pdata[20] = 0x0b;
	uint16_t mycrc = crc_citt_value(pdata, 21);

	uint8_t mycrchighbyte = (uint8_t)(mycrc >> 8);
	uint8_t mycrclowbyte = (uint8_t)mycrc;

	printf("%02x %02x\r\n",mycrchighbyte,mycrclowbyte);

	ObcMemFree(pdata);
	return CMD_ERROR_NONE;
}

static uint8_t test[4];

int ts_cam(struct command_context * context __attribute__((unused))){
    uint32_t test1 = 0x12345678;
    *(uint32_t *)test = test1;

    for(int i=0; i<4; i++)
        printf("%#02x ", test[i]);
    printf("\r\n");
    printf("%#08x\n", *(uint32_t *)test);

    memset(test, 0, (size_t)4);
    test[0] = 0x78;
    test[1] = 0x56;
    test[2] = 0x34;
    test[3] = 0x12;
    printf("%#08x\n", *(uint32_t *)test);

    memset(test, 0, (size_t)4);
    test[0] = 0x12;
    test[1] = 0x34;
    test[2] = 0x56;
    test[3] = 0x78;
    printf("%#08x\n", *(uint32_t *)test);

    printf("ram_persist_start :\t%010p\r\n", &__ram_persist_start);
    printf("ram_persist_end :\t%010p\r\n", &__ram_persist_end);
    printf("__vectors_start :\t%010p\r\n", &__vectors_start);
    printf("__exidx_start :\t\t%010p\r\n", &__exidx_start);
    printf("__exidx_end :\t\t%010p\r\n", &__exidx_end);
    printf("__etext :\t\t%010p\r\n", &__etext);
//    printf("_hk_absolute :\t\t%010p\r\n", &_hk_absolute);
//    printf("_hk_lma :\t\t%010p\r\n", &_hk_lma);
//    printf("__hk_start__ :\t\t%010p\r\n", &__hk_start__);
//    printf("__hk_end__ :\t\t%010p\r\n", &__hk_end__);
//    printf("_data_lma :\t\t%010p\r\n", &_data_lma);
    printf("__data_start__ :\t%010p\r\n", &__data_start__);
    printf("__data_end__ :\t\t%010p\r\n", &__data_end__);
//    printf("_ccr_lma :\t\t%010p\r\n", &_ccr_lma);
//    printf("__ccr_start__ :\t\t%010p\r\n", &__ccr_start__);
//    printf("__ccr_end__ :\t\t%010p\r\n", &__ccr_end__);
    printf("__bss_start__ :\t\t%010p\r\n", &__bss_start__);
    printf("__bss_end__ :\t\t%010p\r\n", &__bss_end__);
    printf("_noinit :\t\t%010p\r\n", &_noinit);
    printf("_end_noinit :\t\t%010p\r\n", &_end_noinit);
    printf("__heap_start__ :\t%010p\r\n", &__heap_start__);

    return CMD_ERROR_NONE;
}

//int Camera_Test (struct command_context * context __attribute__((unused)))
//{
//    ImageQuality MyQuality = highQuality;
//    ImageResolution MyResolution = highResolution;
////    Camera_Synchronize();
////    Camera_Set_Image_Quality(lowQuality);
////    Camera_Set_Image_Resolution(highResolution);
////    Camera_Imaging(0);
////    Camera_Image_Info_Read(0);
////    uint32_t DataRemaining = 0;
////    DataRemaining = ImageSize;
////    for (int i=0; i<HowManyPack; i++, DataRemaining-=506)
////    {
////        if(DataRemaining > 506)
////        {
////            Camera_Get_Image_Packet(i, 506);
////            Camera_Image_Packet_Store(0, i, 506);
////        }
////        else
////        {
////            Camera_Get_Image_Packet(i, DataRemaining);
////            Camera_Image_Packet_Store(0, i, DataRemaining);
////        }
////    }
//    Camera_JPG_Store(0, MyQuality, MyResolution);
//    return 0;
//}

int dtb_tc(struct command_context *ctx )
{
    char * args = command_args(ctx);
    uint8_t cmd;
    if(sscanf(args, "%u", &cmd) != 1)
        return CMD_ERROR_SYNTAX;

    if(!xDTBTeleControlSend(cmd, 1000))
    {
        printf("DTB tc send success!\r\n");
    }

    return CMD_ERROR_NONE;
}

int dtb_tm(struct command_context * context __attribute__((unused)))
{

    dtb_tm_pack * tm = ObcMemMalloc(sizeof(dtb_tm_pack));
    if (tm == NULL)
        return CMD_ERROR_NONE;

    if (xDTBTelemetryGet((uint8_t *)tm, 200) > 0)
    {
        printf("DTB tm receive success!\r\n\r\n");
        dtb_tm_print(tm);
    }
    else
        printf("DTB tm receive fail!\r\n\r\n");

    ObcMemFree(tm);
    return CMD_ERROR_NONE;
}

int ttc_send_cmd(uint8_t handle, uint8_t cmd, uint8_t slen, uint8_t rlen)
{
    uint8_t *pbuffer = ObcMemMalloc((slen>rlen)?slen:rlen);

    if (slen > 3)
        rlen = 0;
    else
    {
        slen = 3;
        if(rlen < 3)
            rlen = 3;
    }

    pbuffer[0] = 0;
    pbuffer[1] = slen-2;
    pbuffer[2] = cmd;

    if (slen>3)
    {
        for (int i=3; i<slen; i++)
        {
            pbuffer[i] = i;
        }
    }

    i2c_master_transaction(handle, 0x18, pbuffer, slen, pbuffer, rlen, 1000);

//    if(rlen)
//    {
//        for(int i=0; i<rlen; i++)
//            printf("%u ", pbuffer[i]);
//        printf("\r\n");
//    }

    ObcMemFree(pbuffer);

    return 0;
}

int ttc_cmd(struct command_context *ctx)
{
    static uint32_t cmd, slen, rlen, ctime;

    char * args = command_args(ctx);
    if(sscanf(args, "%u %u %u %u", &cmd, &slen, &rlen, &ctime) != 4)
        return CMD_ERROR_SYNTAX;

    do
    {
        ttc_send_cmd(0, (uint8_t)cmd, (uint8_t)slen, (uint8_t)rlen);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    } while(--ctime);

    return CMD_ERROR_NONE;
}

int route_queue_send_pack(struct command_context *ctx)
{
    static uint8_t dst, src, typ, len;

    char * args = command_args(ctx);

    if(sscanf(args, "%u %u %u %u", &dst, &src, &typ, &len) != 4)
        return CMD_ERROR_SYNTAX;

    if (len > 232)
        return CMD_ERROR_SYNTAX;

    route_packet_t *packet = (route_packet_t *)ObcMemMalloc(sizeof(route_packet_t)+len);

    packet->len = len;
    packet->dst = dst;
    packet->src = src;
    packet->typ = typ;

    for(int i=0; i<len; i++)
        packet->dat[i] = i+1;

    route_queue_wirte(packet, NULL);

    return CMD_ERROR_NONE;
}

int CubeUnPacket_test(struct command_context *ctx)
{
    uint8_t cmd_type;

    char * args = command_args(ctx);
    if(sscanf(args, "%u", &cmd_type) != 1)
        return CMD_ERROR_SYNTAX;

    route_packet_t *packet = (route_packet_t *)ObcMemMalloc(sizeof(route_packet_t)+10);

    packet->len = 10;
    packet->dst = router_get_my_address();
    packet->src = GND_ROUTE_ADDR;
    packet->typ = cmd_type;

    for(int i=0; i<10; i++)
        packet->dat[i] = 0;

    route_queue_wirte(packet, NULL);

    return CMD_ERROR_NONE;
}

int cam_related_test(struct command_context *ctx __attribute__((unused)))
{
//    uint8_t cmd_type;
//
//    char * args = command_args(ctx);
//    if(sscanf(args, "%u", &cmd_type) != 1)
//        return CMD_ERROR_SYNTAX;
    ImageInfo_t t;
    uint32_t exp_time = 0x00aabbcc;
    Camera_Exposure_Time_Set(exp_time);

    printf("%d\n", sizeof(cam_ctl_t));
    printf("%d\n", sizeof(t.ImageLocation));

    uint8_t test[3];
    memset(test, 0, 3);
    ((cam_ctl_t *)test)->tran = TTL;
    ((cam_ctl_t *)test)->mode = Backup;
    ((cam_ctl_t *)test)->expo = AutoExpoOn;

    return CMD_ERROR_NONE;
}

int cmd_obc_hk(struct command_context *ctx __attribute__((unused)))
{
    obc_hk_t * obc = (obc_hk_t *)ObcMemMalloc(sizeof(obc_hk_t));
    if (obc == NULL)
        return CMD_ERROR_SYNTAX;

    if (obc_hk_get_peek(obc) != pdTRUE)
    {
        ObcMemFree(obc);
        return CMD_ERROR_SYNTAX;
    }

    printf("Item\t\tValue\n");
    printf("******************************\n");
    printf("sat_id\t\t%u\n", obc->sat_id);
    printf("soft_id\t\t%u\n", obc->soft_id);
    printf("reboot_count\t%u\n", obc->reboot_count);
    printf("rec_cmd_count\t%u\n", obc->rec_cmd_count);
    printf("hk_down_count\t%u\n", obc->hk_down_count);
    printf("hk_store_count\t%u\n", obc->hk_store_count);
    printf("i2c_error_count\t%u\n", obc->i2c_error_count);
    printf("last_reset\t%u\n", obc->last_reset_time);
    printf("work_mode\t%u\n", obc->work_mode);
    printf("utc_time\t%u\n", obc->utc_time);
    printf("tmep_mcu\t%.2f\n", obc->tmep_mcu / 100.0);
    printf("tmep_board\t%u\n", obc->tmep_board);
    printf("on_off_status\t0x%08x\n", obc->on_off_status);
    printf("mindex\t\t%u\n", obc->mindex);
    printf("aindex\t\t%u\n", obc->aindex);

    ObcMemFree(obc);
    return CMD_ERROR_NONE;
}

struct command test_subcommands[] = {
	{
		.name = "wflansh",
		.help = "write flansh sector_8",
		.usage = "<index><id><w_len>",
		.handler = I2C_SendToADChip,
	},
//	{
//		.name = "savefiles",
//		.help = "save the audio files in flash sector_8",
//		.handler = save_files,
//	},
	{
		.name = "rpflansh",
		.help = "read flansh sector_8",
		.usage = "<index><id><r_len>",
		.handler = read_Permanent_flansh,
	},
	{
		.name = "rnflansh",
		.help = "read flansh sector_9",
		.usage = "<index><id><r_len>",
		.handler = read_new_flansh,
	},
	{
		.name = "cwpflansh",
		.help = "continuously write flansh sector_8",
		.usage = "<index><id><w_len><Number of frames>",
		.handler = continuously_write_Permanent_flansh,
	},
	{
		.name = "cwnflansh",
		.help = "continuously write flansh sector_9",
		.usage = "<index><id><w_len><Number of frames>",
		.handler = continuously_write_new_flansh,
	},
	{
		.name = "crc16",
		.help = "CameraSynchronize",
		.handler = crc16_generate,
	},
    {
        .name = "ts",
        .help = "just a test",
        .handler = ts_cam,
    },
    {
        .name = "tc_dtb",
        .help = "dtb tc cmd",
        .usage = "<cmd>",
        .handler = dtb_tc,
    },
    {
        .name = "tm_dtb",
        .help = "dtb tm cmd",
        .handler = dtb_tm,
    },
    {
        .name = "ttccmd",
        .help = "ttc test",
        .usage = "<cmd><slen><rlen><ctime>",
        .handler = ttc_cmd,
    },
    {
        .name = "route",
        .help = "send route packet to route queue",
        .usage = "<dst><src><typ><len>",
        .handler = route_queue_send_pack,
    },
    {
        .name = "cmd",
        .help = "UnPacket function test",
        .usage = "<typ>",
        .handler = CubeUnPacket_test,
    },
    {
        .name = "cam",
        .help = "Camera related test",
        .handler = cam_related_test,
    },
    {
        .name = "obc",
        .help = "obc hk print",
        .handler = cmd_obc_hk,
    }
};

struct command __root_command test_commands_master[] =
{
	{
		.name = "test",
		.help = "test commands",
		.chain = INIT_CHAIN(test_subcommands),
	},
};

void cmd_test_setup(void)
{
	command_register(test_commands_master);
}
