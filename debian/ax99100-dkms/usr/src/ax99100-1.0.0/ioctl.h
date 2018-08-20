/* Definition for IOCTL */
#define IOCTL_SET_CUSTOM		_IOW(0xD0, 11, int)
#define IOCTL_GET_CUSTOM		_IOR(0xD0, 12, int)
#define IOCTL_GET_PRODUCT		_IOR(0xD0, 13, int)
#define IOCTL_SET_PARAMETER		_IOW(0xD0, 14, int)
#define IOCTL_SET_SAMPLING		_IOW(0xD0, 15, int)

#define IOCTL_GPIO_DIR			_IOW(0xD0, 16, int)
#define IOCTL_GPIO_STATUS		_IOW(0xD0, 17, int)
#define IOCTL_GPIO_OUTPUT		_IOR(0xD0, 18, int)

#define IOCTL_MSR_INPUT			_IOR(0xD0, 19, int)
#define IOCTL_MCR_OUTPUT		_IOR(0xD0, 20, int)
#define IOCTL_SET_FLOW_CONTROL_ENABLE	_IOR(0xD0, 21, int)
#define IOCTL_SET_FLOW_CONTROL_DISABLE	_IOR(0xD0, 22, int)
#define IOCTL_MCR_INPUT			_IOR(0xD0, 23, int)

#define IOCTL_DMA_RX_OUTPUT		_IOR(0xD0, 24, int)
#define IOCTL_DMA_TX_OUTPUT		_IOR(0xD0, 25, int)

#define IOCTL_DUMP_ALL_REG		_IOR(0xD0, 26, int)


#define DLL_MASK		0xFFFFF00F
#define DLM_MASK		0xFFF00FFF
#define SAMPLE_MASK		0xFFFFFFF0
#define BASE_CLOCK_MASK		0xF00FFFFF

#define CLK_125M	1
#define CLK_1_8382M	0
