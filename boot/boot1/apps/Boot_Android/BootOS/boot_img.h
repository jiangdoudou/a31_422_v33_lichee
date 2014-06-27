/*
*************************************************************************************
*                         			eGon
*					        Application Of eGon2.0
*
*				(c) Copyright 2006-2010, All winners Co,Ld.
*							All	Rights Reserved
*
* File Name 	: BootMain_Private.h
*
* Author 		: javen
*
* Description 	: ��������˽�����ݽṹ
*
* History 		:
*      <author>    		<time>       	<version >    		<desc>
*       javen     	   2010-09-10          1.0            create this file
*
*************************************************************************************
*/
#ifndef  __BOOT_IMG__
#define  __BOOT_IMG__


#define    BOOT_OS_MELIS                1
#define    BOOT_OS_LINUX                2
#define    BOOT_OS_WINCE                3
#define    BOOT_OS_ANDROID              4


typedef struct _boot_img_cfg
{
    char  file_name[20];
    char  file_name_bk[20];
    __u32 base_addr;
    __u32 used_size;
    __u32 full_size;
}
boot_img_cfg_t;

typedef struct boot_img_t
{
    char               img_name[32];         //32���ֽ����������ļ����ƣ�������ȫ·��
    __u32              img_size;             //������󳤶�
    __u32              img_full_size;        //����ʵ�ʳ���
    __u32              img_base;             //�����ŵ��ڴ��е�λ��
}
boot_img_set_t;

typedef struct  boot_sys_img_t
{
    char               os_name[32];          //OS����
    __s32              start_mode;           //����ģʽ������ 0:���վ����ļ��ķ�ʽ���� 1:���շ������ݵķ�ʽ����
    boot_img_set_t     img_set[16];          //ָ����
    boot_img_set_t     script_info;          //ָ��ű���Ϣ
    __u32              img_count;            //��ǰOS��ӵ�еľ������
    char               logo_name[32];        //������ʾlogo��·��
    __s32              logo_show;            //�Ƿ���Ҫ��ʾ����LOGO
    __u32              logo_address;         //����logo��ַ
    __s32			   logo_off;             //����ϵͳǰ�Ƿ���Ҫ�ر�LOGO
}
boot_sys_img_set_t;

typedef struct boot_global_info
{
    __s32              os_count;             //�ܹ���OS����
    char               os_name[8][32];       //OS����, 0����û��ѡ��1����MELIS��2����LINUX��3����WINCE
    char               start_os_name[32];    //��ǰ������OS������
    __s32              start_os_index;       //��ǰ������OS���±�
    __s32              last_os_name;         //�ϴ�������OS
    __s32              user_set_timeout;     //����Ƕ�ϵͳ����������ϵͳѡ�����ʱʱ��
    __s32              display_device;       //��ʾ��LCD��TV��HDMI��
    __s32              display_mode;         //��ʾģʽ
    pic_name_info_t    os_pic[8];            //OS
}
boot_global_info_t;

/* Android bootimage file format */
#define FASTBOOT_BOOT_MAGIC "ANDROID!"
#define FASTBOOT_BOOT_MAGIC_SIZE 8
#define FASTBOOT_BOOT_NAME_SIZE 16
#define FASTBOOT_BOOT_ARGS_SIZE 512

#ifndef CFG_FASTBOOT_MKBOOTIMAGE_PAGE_SIZE
#define CFG_FASTBOOT_MKBOOTIMAGE_PAGE_SIZE 2048
#endif

struct fastboot_boot_img_hdr {
	unsigned char magic[FASTBOOT_BOOT_MAGIC_SIZE];

	unsigned kernel_size;  /* size in bytes */
	unsigned kernel_addr;  /* physical load addr */

	unsigned ramdisk_size; /* size in bytes */
	unsigned ramdisk_addr; /* physical load addr */

	unsigned second_size;  /* size in bytes */
	unsigned second_addr;  /* physical load addr */

	unsigned tags_addr;    /* physical addr for kernel tags */
	unsigned page_size;    /* flash page size we assume */
	unsigned unused[2];    /* future expansion: should be 0 */

	unsigned char name[FASTBOOT_BOOT_NAME_SIZE]; /* asciiz product name */

	unsigned char cmdline[FASTBOOT_BOOT_ARGS_SIZE];

	unsigned id[8]; /* timestamp / checksum / sha1 / etc */
};


#define IH_NMLEN		32	/* Image Name Length		*/
/*
 * Legacy format image header,
 * all data in network byte order (aka natural aka bigendian).
 */
typedef struct image_header {
	uint32_t	ih_magic;	/* Image Header Magic Number	*/
	uint32_t	ih_hcrc;	/* Image Header CRC Checksum	*/
	uint32_t	ih_time;	/* Image Creation Timestamp	*/
	uint32_t	ih_size;	/* Image Data Size		*/
	uint32_t	ih_load;	/* Data	 Load  Address		*/
	uint32_t	ih_ep;		/* Entry Point Address		*/
	uint32_t	ih_dcrc;	/* Image Data CRC Checksum	*/
	uint8_t		ih_os;		/* Operating System		*/
	uint8_t		ih_arch;	/* CPU architecture		*/
	uint8_t		ih_type;	/* Image Type			*/
	uint8_t		ih_comp;	/* Compression Type		*/
	uint8_t		ih_name[IH_NMLEN];	/* Image Name		*/
} image_header_t;


#define ALIGN(x,a)    (((x)+(a)-1)&(~(a-1)))
//#define __ALIGN_MASK(x,mask)	(((x)+(mask))&~(mask))
//#define ALIGN(x,a)		__ALIGN_MASK((x),(typeof(x))(a)-1)


#endif   //__BOOT_IMG__
