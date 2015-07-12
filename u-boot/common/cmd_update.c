/* add by cjcheng for display_param start ...*/
/*
 * (C) Copyright 2002
 * Detlev Zundel, DENX Software Engineering, dzu@denx.de.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * BMP handling routines
 */

#include <common.h>
#include <linux/ctype.h>
#include <command.h>
#include <asm/byteorder.h>

void reverse(char *p)
{
	int len = strlen(p);	
	int i;
	char tmp;
	for (i = 0; i < len/2; i++) {
		tmp = *(p+i);
		*(p+i) = *(p+len-i-1);
		*(p+len-i-1) = tmp;
	}
}

int parser_hextoi(const char *p)
{
	int i = 0, j=0, ret = 0;
	if (p == NULL || strcmp(p, "") == 0)
		printf("%s:%d error !\n", __func__, __LINE__);
	while (isxdigit(*p)){
		j *= 16;
		if (*p <= 'F' && *p >= 'A')
			i = *p++ - 'A' + 10;
		else if (*p <= 'f' && *p >= 'a')
			i = *p++ - 'a' + 10;
		else
			i = *p++ - '0';
		if (j == 0)
			j=1;
		ret += j*i;
	}

	return ret;
}

void check_smdt_param(char *p)  //by jiangdou
{
	char tmp[1024];
	char *s = p;
	int i=0;

	while(*s != '\0'){
		if (*s == '\r' || *s == '\t' || *s == '\n' || *s == ' ' || *s == '\\'){
			s++;
			continue;
		}

		tmp[i++] = *s;
		s++;
	}
	tmp[i] ='\0';
	strcpy(p, tmp);
}

int update_env(void)
{
	uchar str[64];
	uchar smdt_param[1024];
	uchar smdt_param_size[16];
	char *s = NULL;

	memset(smdt_param, 0, sizeof(smdt_param));
    if (storage_type == 0) {
        sprintf(str, "mmc rescan");
        if (!run_command(str, 0)){
            printf("can not find sd card\n");
            return 1;
        }
        sprintf(str, "fatload mmc 0:0 0x%x %s", smdt_param, "/display_param.cfg");//读取SD卡“display_param.cfg”
        if (run_command(str, 0)){
            sprintf(str, "fatload mmc 0:1 0x%x %s", smdt_param, "/display_param.cfg");
            if (run_command(str, 0)){
                printf("can not find param file\n");
                return 1;
            }
        }
    }else if (storage_type == 2){
        sprintf(str, "mmc rescan 3");
        if (!run_command(str, 0)){
            printf("can not find sd card\n");
            return 1;
        }
        sprintf(str, "fatload mmc 3:0 0x%x %s", smdt_param, "/display_param.cfg");
        if (run_command(str, 0)){
            sprintf(str, "fatload mmc 3:1 0x%x %s", smdt_param, "/display_param.cfg");
            if (run_command(str, 0)){
                printf("can not find param file\n");
                return 1;
            }
        }
    }
	check_smdt_param(smdt_param);

	s = getenv("smdt_param");
	if (s == NULL) {
		printf("getenv smdt_param error\n");
		return 1;
	}else {
		if (memcmp(s, smdt_param, strlen(smdt_param)) == 0){
			printf("smdt_param not change\n");
			return 1;
		}else{
			printf("smdt_param change\n");
			setenv("smdt_param", smdt_param);//保存env
			saveenv();
		}
	}
	return 0;
}

int do_update(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	ulong addr;

	if(argc != 1) {
		cmd_usage(cmdtp);
		return 1;
	}
	if (update_env())
		printf("update smdt param file error\n");
	else
		printf("update smdt param file success\n");

	return 0;
}

U_BOOT_CMD(
	update,	1,	1,	do_update,
	"update ",
	"update uboot\n"
);
/* add by cjcheng for display_param end ...*/
