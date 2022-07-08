#define ARM_STD_SVC_MR   0x8400FF80
#define MAX_DESSIZE      50

struct regisiter_nodes {
	unsigned long register_base;	/* 寄存器地址 */
	unsigned long register_offset;	/* 寄存器偏移 */
	unsigned long expected_value;	/* 期望数值 */
	unsigned long bit_mask;	/* 寄存器偏移*/
	unsigned int register_width;	/* 寄存器宽度 32 or 64 */
	unsigned int from_bit;	/* 寄存器bit偏移量 */
	unsigned int to_bit;	/* 寄存器bit偏移量结尾 */
	char description[MAX_DESSIZE];	/* 描述寄存器信息 */
};
