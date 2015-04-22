/*
 * Copyright (C) 2011-2013 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/iomux-mx6dl.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"

/*
#define MX6Q_SABRELITE_SD3_CD		IMX_GPIO_NR(7, 0)
#define MX6Q_SABRELITE_SD3_WP		IMX_GPIO_NR(7, 1)
#define MX6Q_SABRELITE_SD4_CD		IMX_GPIO_NR(2, 6)
#define MX6Q_SABRELITE_SD4_WP		IMX_GPIO_NR(2, 7)
#define MX6Q_SABRELITE_ECSPI1_CS1	IMX_GPIO_NR(3, 19)
#define MX6Q_SABRELITE_USB_OTG_PWR	IMX_GPIO_NR(3, 22)
#define MX6Q_SABRELITE_CAP_TCH_INT1	IMX_GPIO_NR(1, 9)
#define MX6Q_SABRELITE_USB_HUB_RESET	IMX_GPIO_NR(7, 12)
#define MX6Q_SABRELITE_CAN1_STBY	IMX_GPIO_NR(1, 2)
#define MX6Q_SABRELITE_CAN1_EN		IMX_GPIO_NR(1, 4)
#define MX6Q_SABRELITE_MENU_KEY		IMX_GPIO_NR(2, 1)
#define MX6Q_SABRELITE_BACK_KEY		IMX_GPIO_NR(2, 2)
#define MX6Q_SABRELITE_ONOFF_KEY	IMX_GPIO_NR(2, 3)
#define MX6Q_SABRELITE_HOME_KEY		IMX_GPIO_NR(2, 4)
#define MX6Q_SABRELITE_VOL_UP_KEY	IMX_GPIO_NR(7, 13)
#define MX6Q_SABRELITE_VOL_DOWN_KEY	IMX_GPIO_NR(4, 5)
#define MX6Q_SABRELITE_CSI0_RST		IMX_GPIO_NR(1, 8)
#define MX6Q_SABRELITE_CSI0_PWN		IMX_GPIO_NR(1, 6)
*/
//#ifdef CONFIG_MX6_ENET_IRQ_TO_GPIO
#define MX6_ENET_IRQ		IMX_GPIO_NR(1, 6)
#define IOMUX_OBSRV_MUX1_OFFSET	0x3c
#define OBSRV_MUX1_MASK			0x3f
#define OBSRV_MUX1_ENET_IRQ		0x9

#define MX6Q_SABRELITE_SD3_WP_PADCFG	(PAD_CTL_PKE | PAD_CTL_PUE |	\
		PAD_CTL_PUS_22K_UP | PAD_CTL_SPEED_MED |	\
		PAD_CTL_DSE_40ohm | PAD_CTL_HYS)


// (Sylvia) added
#define MX6Q_SABRELITE_GPIO_1		IMX_GPIO_NR(1, 1)
#define MX6Q_SABRELITE_GPIO_2		IMX_GPIO_NR(1, 2)
#define MX6Q_SABRELITE_GPIO_4		IMX_GPIO_NR(1, 4)
#define MX6Q_SABRELITE_GPIO_5		IMX_GPIO_NR(1, 5)
#define MX6Q_SABRELITE_GPIO_7		IMX_GPIO_NR(1, 7)
#define MX6Q_SABRELITE_GPIO_8		IMX_GPIO_NR(1, 8)
#define MX6Q_SABRELITE_GPIO_9		IMX_GPIO_NR(1, 9)
#define MX6Q_SABRELITE_CSI0_PWN		IMX_GPIO_NR(1, 16)
#define MX6Q_SABRELITE_CSI0_RST		IMX_GPIO_NR(1, 17)
#define MX6Q_SABRELITE_PCIE_CSI_PWN	IMX_GPIO_NR(1, 19)
#define MX6Q_SABRELITE_ENET_PHY_INT	IMX_GPIO_NR(1, 28) // copied from boundary

#define MX6Q_SABRELITE_SD2_CD		IMX_GPIO_NR(2, 2)

#define MX6Q_SABRELITE_USB_123_EN	IMX_GPIO_NR(3, 0)
#define MX6Q_SABRELITE_ECSPI1_CS1  	IMX_GPIO_NR(3, 19)  // ECSPI1_SS1 -> GPIO
//#define MX6Q_SABRELITE_USB_OTG_OC	IMX_GPIO_NR(3, 21)  
#define MX6Q_SABRELITE_USB_OTG_PWR	IMX_GPIO_NR(3, 22)  // USB_OTG_PWR -> GPIO (X : NO USE OTG POWER PIN)
#define MX6Q_SABRELITE_LVDS_DE		IMX_GPIO_NR(3, 28)  // 0/X
//#define MX6Q_SABRELITE_USB_123_OC	IMX_GPIO_NR(3, 30)  

#define MX6Q_SABRELITE_GPIO_19_PLED	IMX_GPIO_NR(4, 5)
/* Ken modified for VAB820 RA PIN changed */
//#define MX6Q_SABRELITE_PCIE_DIS_B	IMX_GPIO_NR(4, 14)
#define MX6Q_SABRELITE_PCIE_DIS_B	IMX_GPIO_NR(4, 9)
#define MX6Q_SABRELITE_ECSPI3_CS0  	IMX_GPIO_NR(4, 24)  // CSPI3_CS0 -> GPIO
#define MX6Q_SABRELITE_ECSPI3_CS1  	IMX_GPIO_NR(4, 25)  // CSPI3_CS1 -> GPIO

#define MX6Q_SABRELITE_PCIE_WAKE_B	IMX_GPIO_NR(5, 20)

#define MX6Q_SABRELITE_CAP_TCH_INT0	IMX_GPIO_NR(6, 8)
#define MX6Q_SABRELITE_CABC_EN0		IMX_GPIO_NR(6, 15)  // LVDS_EN

#define MX6Q_SABRELITE_GPIO_16		IMX_GPIO_NR(7, 11)
#define MX6Q_SABRELITE_PCIE_RST_B	IMX_GPIO_NR(7, 12)
#define MX6Q_SABRELITE_USB_HUB_RST_B	IMX_GPIO_NR(7, 13)


void __init early_console_setup(unsigned long base, struct clk *clk);
static struct clk *sata_clk;

extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;
extern bool enet_to_gpio_6;
static int caam_enabled;

extern struct regulator *(*get_cpu_regulator)(void);
extern void (*put_cpu_regulator)(void);

static iomux_v3_cfg_t mx6q_sabrelite_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6Q_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6Q_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6Q_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk - GPIO_0_CLKO */

	/* HDMI  */
	MX6Q_PAD_KEY_ROW2__HDMI_TX_CEC_LINE, // HDMI_CEC_IN 
	
	/* ECSPI1 */
	// (Sylvia) added
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	//MX6Q_PAD_EIM_D19__ECSPI1_SS1, 
					// (what's the diff between GPIO and ESCPI1_SS1???)
					// both SDB and SabreLite choose GPIO rather then ECSPI1_SS1
	/* ECSPI3 */	
	// (Sylvia) added
	MX6Q_PAD_DISP0_DAT0__ECSPI3_SCLK,
	MX6Q_PAD_DISP0_DAT1__ECSPI3_MOSI,
	MX6Q_PAD_DISP0_DAT2__ECSPI3_MISO,
	MX6Q_PAD_DISP0_DAT3__GPIO_4_24,
	MX6Q_PAD_DISP0_DAT4__GPIO_4_25,


#if 1   // VAB820
	/* ENET */
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,

	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,	// ENET_REF_CLK
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */ // RGMII_INT : ENET_TX_EN(ALT5)
	
	// (Sylvia) added
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25,		// RGMII reset - RGMII_nRST : ENET_CRS_DV(ALT5)
#endif

	/* GPIO pins */ // (Sylvia) added
	MX6Q_PAD_GPIO_1__GPIO_1_1,	// GPIO1
	MX6Q_PAD_GPIO_2__GPIO_1_2,	// GPIO2
	MX6Q_PAD_GPIO_4__GPIO_1_4,	// GPIO4
	MX6Q_PAD_GPIO_5__GPIO_1_5,	// GPIO5
	MX6Q_PAD_GPIO_7__GPIO_1_7,	// GPIO7
	MX6Q_PAD_GPIO_8__GPIO_1_8,	// GPIO8
	MX6Q_PAD_GPIO_9__GPIO_1_9,	// GPIO9
	MX6Q_PAD_GPIO_16__GPIO_7_11,	// GPIO16


	/* GPIO4 */ // (Sylvia) added
	MX6Q_PAD_GPIO_19__GPIO_4_5,	// GPIO_19_PLED : for LED turn off

	/* I2C1 - ADV7180 */
	MX6Q_PAD_CSI0_DAT8__I2C1_SDA,
	MX6Q_PAD_CSI0_DAT9__I2C1_SCL,

	/* I2C2 - HDMI, Audio */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 - LVDS, PCIE, J7 */
	MX6Q_PAD_GPIO_3__I2C3_SCL,
	MX6Q_PAD_GPIO_6__I2C3_SDA,


	/* CSI - ADV7180 */ 
	// (Sylvia) added
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	MX6Q_PAD_SD1_DAT0__GPIO_1_16,	// CSI0_PWN
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,	// CSI0_RST_B



	/* LVDS */
	MX6Q_PAD_NANDF_ALE__GPIO_6_8,	// CAP_TCH_INT0
	MX6Q_PAD_NANDF_CS2__GPIO_6_15,	// LVDS_EN
	MX6Q_PAD_EIM_D28__GPIO_3_28,	// LVDS_DE
	MX6Q_PAD_DISP0_DAT9__PWM2_PWMO,	// DISP0_CONTRAST : PWM2_OUT(ALT2)

	/* PCIE */
	MX6Q_PAD_GPIO_17__GPIO_7_12, 		/* PCIE_RST_B */
	/* Ken modified for VAB820 RA PIN changed */
	//MX6Q_PAD_KEY_COL4__GPIO_4_14, 	/* PCIE_DIS_B */
	MX6Q_PAD_KEY_ROW1__GPIO_4_9,		/* PCIE_DIS_B */
	MX6Q_PAD_CSI0_DATA_EN__GPIO_5_20,	// PCIE_WAKE_B
	MX6Q_PAD_SD1_DAT2__GPIO_1_19,		// CSI_PWN

	/* UART1 */
	// (Sylvia) added : DTE mode
	MX6Q_PAD_SD3_DAT6__UART1_TXD, 	// IOMUX_PAD(0x0694, 0x02AC, 1, 0x0000, 0, 0)
	MX6Q_PAD_SD3_DAT7__UART1_RXD,	// IOMUX_PAD(0x0690, 0x02A8, 1, 0x0920, 2, 0) // assign RX => 2: SD3_DAT7
	MX6Q_PAD_EIM_D20__UART1_CTS,	// IOMUX_PAD(0x03B4, 0x00A0, 4, 0x0000, 0, 0)
	MX6Q_PAD_SD3_DAT0__UART1_RTS,	// IOMUX_PAD(0x06A8, 0x02C0, 1, 0x091C, 2, 0) // assign RTS => 2 : SD3_DAT0
	//MX6Q_PAD_EIM_D24__UART1_DTR,	// (Peter) modify for strange issue
	MX6Q_PAD_EIM_D24__GPIO_3_24,
	MX6Q_PAD_EIM_D25__UART1_DSR,
	MX6Q_PAD_EIM_D23__UART1_DCD,
	MX6Q_PAD_EIM_EB3__UART1_RI,

	/* UART2 for debug */
	// (Sylvia) added
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,	

	/* USB OTG*/
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,	// USB_OTG_ID
	MX6Q_PAD_EIM_D21__USBOH3_USBOTG_OC,	//MX6Q_PAD_EIM_D21__GPIO_3_21,		// USB_OTG_OC
	MX6Q_PAD_EIM_D22__GPIO_3_22, 		// (Sylvia) why GPIO for power pin???
	//(Sylvia) added
	//MX6Q_PAD_EIM_D22__USBOH3_USBOTG_PWR,	// USB_OTG_PWR_EN // should it be defined as GPIO ??? (ref to SabreLite)

	/* USB */
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,	// MX6Q_PAD_EIM_D30__GPIO_3_30,		// USB_123_OC
	MX6Q_PAD_EIM_DA0__GPIO_3_0,		// USB_123_EN (0:enable 1:disable)
	MX6Q_PAD_GPIO_18__GPIO_7_13,		// USB_HUB_RESET_B

	/* USDHC2 - Micro SD */
	MX6Q_PAD_SD2_CLK__USDHC2_CLK,
	MX6Q_PAD_SD2_CMD__USDHC2_CMD,
	MX6Q_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6Q_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6Q_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6Q_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6Q_PAD_NANDF_D2__GPIO_2_2,		/* SD2_CD */

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6Q_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6Q_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6Q_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,

	/* WDOG-1 */
	MX6Q_PAD_DISP0_DAT8__WDOG1_WDOG_B,

	/* Ken added CAN1, CAN2 for VAB820 RA */
        /* CAN1 */
        MX6Q_PAD_SD3_CMD__CAN1_TXCAN,
        MX6Q_PAD_SD3_CLK__CAN1_RXCAN,
        /* CAN2 */
        MX6Q_PAD_KEY_COL4__CAN2_TXCAN,
        MX6Q_PAD_KEY_ROW4__CAN2_RXCAN,

};

#if 0
static iomux_v3_cfg_t mx6q_sabrelite_pads[] = {
	/* AUDMUX */
	MX6Q_PAD_SD2_DAT0__AUDMUX_AUD4_RXD,
	MX6Q_PAD_SD2_DAT3__AUDMUX_AUD4_TXC,
	MX6Q_PAD_SD2_DAT2__AUDMUX_AUD4_TXD,
	MX6Q_PAD_SD2_DAT1__AUDMUX_AUD4_TXFS,

	/* CAN1  */
	MX6Q_PAD_KEY_ROW2__CAN1_RXCAN,
	MX6Q_PAD_KEY_COL2__CAN1_TXCAN,
	MX6Q_PAD_GPIO_2__GPIO_1_2,		/* STNDBY */
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* NERR */
	MX6Q_PAD_GPIO_4__GPIO_1_4,		/* Enable */

	/* CCM  */
	MX6Q_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk */
	MX6Q_PAD_GPIO_3__CCM_CLKO2,		/* J5 - Camera MCLK */

	/* ECSPI1 */
	MX6Q_PAD_EIM_D17__ECSPI1_MISO,
	MX6Q_PAD_EIM_D18__ECSPI1_MOSI,
	MX6Q_PAD_EIM_D16__ECSPI1_SCLK,
	MX6Q_PAD_EIM_D19__GPIO_3_19,	/*SS1*/

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	MX6Q_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */
	MX6Q_PAD_EIM_D23__GPIO_3_23,		/* RGMII reset */

	/* GPIO1 */
	MX6Q_PAD_ENET_RX_ER__GPIO_1_24,		/* J9 - Microphone Detect */

	/* GPIO2 */
	MX6Q_PAD_NANDF_D1__GPIO_2_1,	/* J14 - Menu Button */
	MX6Q_PAD_NANDF_D2__GPIO_2_2,	/* J14 - Back Button */
	MX6Q_PAD_NANDF_D3__GPIO_2_3,	/* J14 - Search Button */
	MX6Q_PAD_NANDF_D4__GPIO_2_4,	/* J14 - Home Button */
	MX6Q_PAD_EIM_A22__GPIO_2_16,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A21__GPIO_2_17,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A20__GPIO_2_18,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A19__GPIO_2_19,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A18__GPIO_2_20,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A17__GPIO_2_21,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A16__GPIO_2_22,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_RW__GPIO_2_26,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_LBA__GPIO_2_27,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB0__GPIO_2_28,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB1__GPIO_2_29,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_EB3__GPIO_2_31,	/* J12 - Boot Mode Select */

	/* GPIO3 */
	MX6Q_PAD_EIM_DA0__GPIO_3_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA1__GPIO_3_1,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA2__GPIO_3_2,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA3__GPIO_3_3,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA4__GPIO_3_4,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA5__GPIO_3_5,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA6__GPIO_3_6,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA7__GPIO_3_7,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA8__GPIO_3_8,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA9__GPIO_3_9,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA10__GPIO_3_10,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA11__GPIO_3_11,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA12__GPIO_3_12,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA13__GPIO_3_13,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA14__GPIO_3_14,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_DA15__GPIO_3_15,	/* J12 - Boot Mode Select */

	/* GPIO4 */
	MX6Q_PAD_GPIO_19__GPIO_4_5,	/* J14 - Volume Down */

	/* GPIO5 */
	MX6Q_PAD_EIM_WAIT__GPIO_5_0,	/* J12 - Boot Mode Select */
	MX6Q_PAD_EIM_A24__GPIO_5_4,	/* J12 - Boot Mode Select */

	/* GPIO6 */
	MX6Q_PAD_EIM_A23__GPIO_6_6,	/* J12 - Boot Mode Select */

	/* GPIO7 */
	MX6Q_PAD_GPIO_17__GPIO_7_12,	/* USB Hub Reset */
	MX6Q_PAD_GPIO_18__GPIO_7_13,	/* J14 - Volume Up */

	/* I2C1, SGTL5000 */
	MX6Q_PAD_EIM_D21__I2C1_SCL,	/* GPIO3[21] */
	MX6Q_PAD_EIM_D28__I2C1_SDA,	/* GPIO3[28] */

	/* I2C2 Camera, MIPI */
	MX6Q_PAD_KEY_COL3__I2C2_SCL,    /* GPIO4[12] */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,    /* GPIO4[13] */

	/* I2C3 */
	MX6Q_PAD_GPIO_5__I2C3_SCL,	/* GPIO1[5] - J7 - Display card */
#ifdef CONFIG_FEC_1588
	MX6Q_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT,
#else
	MX6Q_PAD_GPIO_16__I2C3_SDA,	/* GPIO7[11] - J15 - RGB connector */
#endif

	/* DISPLAY */
	MX6Q_PAD_DI0_DISP_CLK__IPU1_DI0_DISP_CLK,
	MX6Q_PAD_DI0_PIN15__IPU1_DI0_PIN15,		/* DE */
	MX6Q_PAD_DI0_PIN2__IPU1_DI0_PIN2,		/* HSync */
	MX6Q_PAD_DI0_PIN3__IPU1_DI0_PIN3,		/* VSync */
	MX6Q_PAD_DI0_PIN4__IPU1_DI0_PIN4,		/* Contrast */
	MX6Q_PAD_DISP0_DAT0__IPU1_DISP0_DAT_0,
	MX6Q_PAD_DISP0_DAT1__IPU1_DISP0_DAT_1,
	MX6Q_PAD_DISP0_DAT2__IPU1_DISP0_DAT_2,
	MX6Q_PAD_DISP0_DAT3__IPU1_DISP0_DAT_3,
	MX6Q_PAD_DISP0_DAT4__IPU1_DISP0_DAT_4,
	MX6Q_PAD_DISP0_DAT5__IPU1_DISP0_DAT_5,
	MX6Q_PAD_DISP0_DAT6__IPU1_DISP0_DAT_6,
	MX6Q_PAD_DISP0_DAT7__IPU1_DISP0_DAT_7,
	MX6Q_PAD_DISP0_DAT8__IPU1_DISP0_DAT_8,
	MX6Q_PAD_DISP0_DAT9__IPU1_DISP0_DAT_9,
	MX6Q_PAD_DISP0_DAT10__IPU1_DISP0_DAT_10,
	MX6Q_PAD_DISP0_DAT11__IPU1_DISP0_DAT_11,
	MX6Q_PAD_DISP0_DAT12__IPU1_DISP0_DAT_12,
	MX6Q_PAD_DISP0_DAT13__IPU1_DISP0_DAT_13,
	MX6Q_PAD_DISP0_DAT14__IPU1_DISP0_DAT_14,
	MX6Q_PAD_DISP0_DAT15__IPU1_DISP0_DAT_15,
	MX6Q_PAD_DISP0_DAT16__IPU1_DISP0_DAT_16,
	MX6Q_PAD_DISP0_DAT17__IPU1_DISP0_DAT_17,
	MX6Q_PAD_DISP0_DAT18__IPU1_DISP0_DAT_18,
	MX6Q_PAD_DISP0_DAT19__IPU1_DISP0_DAT_19,
	MX6Q_PAD_DISP0_DAT20__IPU1_DISP0_DAT_20,
	MX6Q_PAD_DISP0_DAT21__IPU1_DISP0_DAT_21,
	MX6Q_PAD_DISP0_DAT22__IPU1_DISP0_DAT_22,
	MX6Q_PAD_DISP0_DAT23__IPU1_DISP0_DAT_23,
	MX6Q_PAD_GPIO_7__GPIO_1_7,		/* J7 - Display Connector GP */
	MX6Q_PAD_GPIO_9__GPIO_1_9,		/* J7 - Display Connector GP */
	MX6Q_PAD_NANDF_D0__GPIO_2_0,		/* J6 - LVDS Display contrast */


	/* PWM1 */
	MX6Q_PAD_SD1_DAT3__PWM1_PWMO,		/* GPIO1[21] */

	/* PWM2 */
	MX6Q_PAD_SD1_DAT2__PWM2_PWMO,		/* GPIO1[19] */

	/* PWM3 */
	MX6Q_PAD_SD1_DAT1__PWM3_PWMO,		/* GPIO1[17] */

	/* PWM4 */
	MX6Q_PAD_SD1_CMD__PWM4_PWMO,		/* GPIO1[18] */

	/* UART1  */
	MX6Q_PAD_SD3_DAT7__UART1_TXD,
	MX6Q_PAD_SD3_DAT6__UART1_RXD,

	/* UART2 for debug */
	MX6Q_PAD_EIM_D26__UART2_TXD,
	MX6Q_PAD_EIM_D27__UART2_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_GPIO_1__USBOTG_ID,

	/* USB OC pin */
	MX6Q_PAD_KEY_COL4__USBOH3_USBOTG_OC,
	MX6Q_PAD_EIM_D30__USBOH3_USBH1_OC,

	/* USDHC3 */
	MX6Q_PAD_SD3_CLK__USDHC3_CLK_50MHZ,
	MX6Q_PAD_SD3_CMD__USDHC3_CMD_50MHZ,
	MX6Q_PAD_SD3_DAT0__USDHC3_DAT0_50MHZ,
	MX6Q_PAD_SD3_DAT1__USDHC3_DAT1_50MHZ,
	MX6Q_PAD_SD3_DAT2__USDHC3_DAT2_50MHZ,
	MX6Q_PAD_SD3_DAT3__USDHC3_DAT3_50MHZ,
	MX6Q_PAD_SD3_DAT5__GPIO_7_0,		/* J18 - SD3_CD */
	NEW_PAD_CTRL(MX6Q_PAD_SD3_DAT4__GPIO_7_1, MX6Q_SABRELITE_SD3_WP_PADCFG),

	/* USDHC4 */
	MX6Q_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6Q_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6Q_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6Q_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6Q_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6Q_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6Q_PAD_NANDF_D6__GPIO_2_6,		/* J20 - SD4_CD */
	MX6Q_PAD_NANDF_D7__GPIO_2_7,		/* SD4_WP */
};
#endif

// (Mike@Avnet) added for DL
static iomux_v3_cfg_t mx6dl_sabrelite_pads[] = {
	/* AUDMUX */
	MX6DL_PAD_DISP0_DAT20__AUDMUX_AUD4_TXC,
	MX6DL_PAD_DISP0_DAT21__AUDMUX_AUD4_TXD,
	MX6DL_PAD_DISP0_DAT22__AUDMUX_AUD4_TXFS,
	MX6DL_PAD_DISP0_DAT23__AUDMUX_AUD4_RXD,

	/* CCM  */
	MX6DL_PAD_GPIO_0__CCM_CLKO,		/* SGTL500 sys_mclk - GPIO_0_CLKO */

	/* HDMI  */
	MX6DL_PAD_KEY_ROW2__HDMI_TX_CEC_LINE, // HDMI_CEC_IN

	/* ECSPI1 */
	// (Sylvia) added
	MX6DL_PAD_EIM_D16__ECSPI1_SCLK,
	MX6DL_PAD_EIM_D17__ECSPI1_MISO,
	MX6DL_PAD_EIM_D18__ECSPI1_MOSI,
	MX6DL_PAD_EIM_D19__GPIO_3_19,	//MX6Q_PAD_EIM_D19__ECSPI1_SS1,
					// (what's the diff between GPIO and ESCPI1_SS1???)
					// both SDB and SabreLite choose GPIO rather then ECSPI1_SS1
	/* ECSPI3 */
	// (Sylvia) added
	MX6DL_PAD_DISP0_DAT0__ECSPI3_SCLK,
	MX6DL_PAD_DISP0_DAT1__ECSPI3_MOSI,
	MX6DL_PAD_DISP0_DAT2__ECSPI3_MISO,
	MX6DL_PAD_DISP0_DAT3__GPIO_4_24,
	MX6DL_PAD_DISP0_DAT4__GPIO_4_25,


#if 1   // VAB820
	/* ENET */
	MX6DL_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6DL_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6DL_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6DL_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6DL_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6DL_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6DL_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6DL_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6DL_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6DL_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6DL_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6DL_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,

	MX6DL_PAD_ENET_MDIO__ENET_MDIO,
	MX6DL_PAD_ENET_MDC__ENET_MDC,
	MX6DL_PAD_ENET_REF_CLK__ENET_TX_CLK,	// ENET_REF_CLK
	MX6DL_PAD_ENET_TX_EN__GPIO_1_28,		/* Micrel RGMII Phy Interrupt */ // RGMII_INT : ENET_TX_EN(ALT5)

	// (Sylvia) added
	MX6DL_PAD_ENET_CRS_DV__GPIO_1_25,		// RGMII reset - RGMII_nRST : ENET_CRS_DV(ALT5)
#endif

	/* GPIO pins */ // (Sylvia) added
	MX6DL_PAD_GPIO_1__GPIO_1_1,	// GPIO1
	MX6DL_PAD_GPIO_2__GPIO_1_2,	// GPIO2
	MX6DL_PAD_GPIO_4__GPIO_1_4,	// GPIO4
	MX6DL_PAD_GPIO_5__GPIO_1_5,	// GPIO5
	MX6DL_PAD_GPIO_7__GPIO_1_7,	// GPIO7
	MX6DL_PAD_GPIO_8__GPIO_1_8,	// GPIO8
	MX6DL_PAD_GPIO_9__GPIO_1_9,	// GPIO9
	MX6DL_PAD_GPIO_16__GPIO_7_11,	// GPIO16


	/* GPIO4 */ // (Sylvia) added
	MX6DL_PAD_GPIO_19__GPIO_4_5,	// GPIO_19_PLED : for LED turn off

	/* I2C1 - ADV7180 */
	MX6DL_PAD_CSI0_DAT8__I2C1_SDA,
	MX6DL_PAD_CSI0_DAT9__I2C1_SCL,

	/* I2C2 - HDMI, Audio */
	MX6DL_PAD_KEY_COL3__I2C2_SCL,
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,

	/* I2C3 - LVDS, PCIE, J7 */
	MX6DL_PAD_GPIO_3__I2C3_SCL,
	MX6DL_PAD_GPIO_6__I2C3_SDA,


	/* CSI - ADV7180 */
	// (Sylvia) added
	MX6DL_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6DL_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6DL_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6DL_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6DL_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6DL_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6DL_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6DL_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6DL_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6DL_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	MX6DL_PAD_SD1_DAT0__GPIO_1_16,	// CSI0_PWN
	MX6DL_PAD_SD1_DAT1__GPIO_1_17,	// CSI0_RST_B



	/* LVDS */
	MX6DL_PAD_NANDF_ALE__GPIO_6_8,	// CAP_TCH_INT0
	MX6DL_PAD_NANDF_CS2__GPIO_6_15,	// LVDS_EN
	MX6DL_PAD_EIM_D28__GPIO_3_28,	// LVDS_DE
	MX6DL_PAD_DISP0_DAT9__PWM2_PWMO,	// DISP0_CONTRAST : PWM2_OUT(ALT2)

	/* PCIE */
	MX6DL_PAD_GPIO_17__GPIO_7_12, 		/* PCIE_RST_B */
	/* Ken modified for VAB820 RA PIN changed */
	//MX6Q_PAD_KEY_COL4__GPIO_4_14, 	/* PCIE_DIS_B */
	MX6DL_PAD_KEY_ROW1__GPIO_4_9,		/* PCIE_DIS_B */
	MX6DL_PAD_CSI0_DATA_EN__GPIO_5_20,	// PCIE_WAKE_B
	MX6DL_PAD_SD1_DAT2__GPIO_1_19,		// CSI_PWN

	/* UART1 */
	// (Sylvia) added : DTE mode
	MX6DL_PAD_SD3_DAT6__UART1_TXD, 	// IOMUX_PAD(0x0694, 0x02AC, 1, 0x0000, 0, 0)
	MX6DL_PAD_SD3_DAT7__UART1_RXD,	// IOMUX_PAD(0x0690, 0x02A8, 1, 0x0920, 2, 0) // assign RX => 2: SD3_DAT7
	MX6DL_PAD_EIM_D20__UART1_CTS,	// IOMUX_PAD(0x03B4, 0x00A0, 4, 0x0000, 0, 0)
	MX6DL_PAD_SD3_DAT0__UART1_RTS,	// IOMUX_PAD(0x06A8, 0x02C0, 1, 0x091C, 2, 0) // assign RTS => 2 : SD3_DAT0
	//MX6Q_PAD_EIM_D24__UART1_DTR,	// (Peter) modify for strange issue
	MX6DL_PAD_EIM_D24__GPIO_3_24,
	MX6DL_PAD_EIM_D25__UART1_DSR,
	MX6DL_PAD_EIM_D23__UART1_DCD,
	MX6DL_PAD_EIM_EB3__UART1_RI,

	/* UART2 for debug */
	// (Sylvia) added
	MX6DL_PAD_EIM_D26__UART2_TXD,
	MX6DL_PAD_EIM_D27__UART2_RXD,

	/* USB OTG*/
	MX6DL_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,	// USB_OTG_ID
	MX6DL_PAD_EIM_D21__USBOH3_USBOTG_OC,	//MX6Q_PAD_EIM_D21__GPIO_3_21,		// USB_OTG_OC
	MX6DL_PAD_EIM_D22__GPIO_3_22, 		// (Sylvia) why GPIO for power pin???
	//(Sylvia) added
	//MX6Q_PAD_EIM_D22__USBOH3_USBOTG_PWR,	// USB_OTG_PWR_EN // should it be defined as GPIO ??? (ref to SabreLite)

	/* USB */
	MX6DL_PAD_EIM_D30__USBOH3_USBH1_OC,	// MX6Q_PAD_EIM_D30__GPIO_3_30,		// USB_123_OC
	MX6DL_PAD_EIM_DA0__GPIO_3_0,		// USB_123_EN (0:enable 1:disable)
	MX6DL_PAD_GPIO_18__GPIO_7_13,		// USB_HUB_RESET_B

	/* USDHC2 - Micro SD */
	MX6DL_PAD_SD2_CLK__USDHC2_CLK,
	MX6DL_PAD_SD2_CMD__USDHC2_CMD,
	MX6DL_PAD_SD2_DAT0__USDHC2_DAT0,
	MX6DL_PAD_SD2_DAT1__USDHC2_DAT1,
	MX6DL_PAD_SD2_DAT2__USDHC2_DAT2,
	MX6DL_PAD_SD2_DAT3__USDHC2_DAT3,
	MX6DL_PAD_NANDF_D2__GPIO_2_2,		/* SD2_CD */

	/* USDHC4 */
	MX6DL_PAD_SD4_CLK__USDHC4_CLK_50MHZ,
	MX6DL_PAD_SD4_CMD__USDHC4_CMD_50MHZ,
	MX6DL_PAD_SD4_DAT0__USDHC4_DAT0_50MHZ,
	MX6DL_PAD_SD4_DAT1__USDHC4_DAT1_50MHZ,
	MX6DL_PAD_SD4_DAT2__USDHC4_DAT2_50MHZ,
	MX6DL_PAD_SD4_DAT3__USDHC4_DAT3_50MHZ,
	MX6DL_PAD_SD4_DAT4__USDHC4_DAT4_50MHZ,
	MX6DL_PAD_SD4_DAT5__USDHC4_DAT5_50MHZ,
	MX6DL_PAD_SD4_DAT6__USDHC4_DAT6_50MHZ,
	MX6DL_PAD_SD4_DAT7__USDHC4_DAT7_50MHZ,

	/* WDOG-1 */
	MX6DL_PAD_DISP0_DAT8__WDOG1_WDOG_B,

	/* Ken added CAN1, CAN2 for VAB820 RA */
        /* CAN1 */
        MX6DL_PAD_SD3_CMD__CAN1_TXCAN,
        MX6DL_PAD_SD3_CLK__CAN1_RXCAN,
        /* CAN2 */
        MX6DL_PAD_KEY_COL4__CAN2_TXCAN,
        MX6DL_PAD_KEY_ROW4__CAN2_RXCAN,

};

static iomux_v3_cfg_t mx6q_sabrelite_csi0_sensor_pads[] = {
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	MX6Q_PAD_SD1_DAT0__GPIO_1_16,	// CSI0_PWN
	MX6Q_PAD_SD1_DAT1__GPIO_1_17,	// CSI0_RST_B

#if 0   // (Sylvia) marked
	/* IPU1 Camera */
	MX6Q_PAD_CSI0_DAT8__IPU1_CSI0_D_8,
	MX6Q_PAD_CSI0_DAT9__IPU1_CSI0_D_9,
	MX6Q_PAD_CSI0_DAT10__IPU1_CSI0_D_10,
	MX6Q_PAD_CSI0_DAT11__IPU1_CSI0_D_11,
	MX6Q_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6Q_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6Q_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6Q_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6Q_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6Q_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6Q_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6Q_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6Q_PAD_CSI0_DATA_EN__IPU1_CSI0_DATA_EN,
	MX6Q_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6Q_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,
	MX6Q_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6Q_PAD_GPIO_8__GPIO_1_8,		/* J5 - Camera Reset */
	MX6Q_PAD_SD1_DAT0__GPIO_1_16,		/* J5 - Camera GP */
	MX6Q_PAD_NANDF_D5__GPIO_2_5,		/* J16 - MIPI GP */
	MX6Q_PAD_NANDF_WP_B__GPIO_6_9,		/* J16 - MIPI GP */
#endif
};

// (Mike@Avnet) added for DL
static iomux_v3_cfg_t mx6dl_sabrelite_csi0_sensor_pads[] = {
	MX6DL_PAD_CSI0_DAT12__IPU1_CSI0_D_12,
	MX6DL_PAD_CSI0_DAT13__IPU1_CSI0_D_13,
	MX6DL_PAD_CSI0_DAT14__IPU1_CSI0_D_14,
	MX6DL_PAD_CSI0_DAT15__IPU1_CSI0_D_15,
	MX6DL_PAD_CSI0_DAT16__IPU1_CSI0_D_16,
	MX6DL_PAD_CSI0_DAT17__IPU1_CSI0_D_17,
	MX6DL_PAD_CSI0_DAT18__IPU1_CSI0_D_18,
	MX6DL_PAD_CSI0_DAT19__IPU1_CSI0_D_19,
	MX6DL_PAD_CSI0_VSYNC__IPU1_CSI0_VSYNC,
	MX6DL_PAD_CSI0_MCLK__IPU1_CSI0_HSYNC,
	MX6DL_PAD_CSI0_PIXCLK__IPU1_CSI0_PIXCLK,

	MX6DL_PAD_SD1_DAT0__GPIO_1_16,	// CSI0_PWN
	MX6DL_PAD_SD1_DAT1__GPIO_1_17,	// CSI0_RST_B
};

static iomux_v3_cfg_t mx6q_sabrelite_hdmi_ddc_pads[] = {
	MX6Q_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6Q_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

// (Mike@Avnet) added for DL
static iomux_v3_cfg_t mx6dl_sabrelite_hdmi_ddc_pads[] = {
	MX6DL_PAD_KEY_COL3__HDMI_TX_DDC_SCL, /* HDMI DDC SCL */
	MX6DL_PAD_KEY_ROW3__HDMI_TX_DDC_SDA, /* HDMI DDC SDA */
};

static iomux_v3_cfg_t mx6q_sabrelite_i2c2_pads[] = {
	MX6Q_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6Q_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

// (Mike@Avnet) added for DL
static iomux_v3_cfg_t mx6dl_sabrelite_i2c2_pads[] = {
	MX6DL_PAD_KEY_COL3__I2C2_SCL,	/* I2C2 SCL */
	MX6DL_PAD_KEY_ROW3__I2C2_SDA,	/* I2C2 SDA */
};

#define MX6Q_USDHC_PAD_SETTING(id, speed)	\
mx6q_sd##id##_##speed##mhz[] = {		\
	MX6Q_PAD_SD##id##_CLK__USDHC##id##_CLK_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_CMD__USDHC##id##_CMD_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT0__USDHC##id##_DAT0_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT1__USDHC##id##_DAT1_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT2__USDHC##id##_DAT2_##speed##MHZ,	\
	MX6Q_PAD_SD##id##_DAT3__USDHC##id##_DAT3_##speed##MHZ,	\
}

// (Sylvia) marked
//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 50);
//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 100);
//static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(3, 200);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 50);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 100);
static iomux_v3_cfg_t MX6Q_USDHC_PAD_SETTING(4, 200);

enum sd_pad_mode {
	SD_PAD_MODE_LOW_SPEED,
	SD_PAD_MODE_MED_SPEED,
	SD_PAD_MODE_HIGH_SPEED,
};

static int plt_sd_pad_change(unsigned int index, int clock)
{
	/* LOW speed is the default state of SD pads */
	static enum sd_pad_mode pad_mode = SD_PAD_MODE_LOW_SPEED;

	iomux_v3_cfg_t *sd_pads_200mhz = NULL;
	iomux_v3_cfg_t *sd_pads_100mhz = NULL;
	iomux_v3_cfg_t *sd_pads_50mhz = NULL;

	u32 sd_pads_200mhz_cnt;
	u32 sd_pads_100mhz_cnt;
	u32 sd_pads_50mhz_cnt;

	switch (index) {
#if 0	// (Sylvia) marked
	case 2:
		sd_pads_200mhz = mx6q_sd3_200mhz;
		sd_pads_100mhz = mx6q_sd3_100mhz;
		sd_pads_50mhz = mx6q_sd3_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd3_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd3_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd3_50mhz);
		break;
#endif
	case 3:
		sd_pads_200mhz = mx6q_sd4_200mhz;
		sd_pads_100mhz = mx6q_sd4_100mhz;
		sd_pads_50mhz = mx6q_sd4_50mhz;

		sd_pads_200mhz_cnt = ARRAY_SIZE(mx6q_sd4_200mhz);
		sd_pads_100mhz_cnt = ARRAY_SIZE(mx6q_sd4_100mhz);
		sd_pads_50mhz_cnt = ARRAY_SIZE(mx6q_sd4_50mhz);
		break;
	default:
		printk(KERN_ERR "no such SD host controller index %d\n", index);
		return -EINVAL;
	}

	if (clock > 100000000) {
		if (pad_mode == SD_PAD_MODE_HIGH_SPEED)
			return 0;
		BUG_ON(!sd_pads_200mhz);
		pad_mode = SD_PAD_MODE_HIGH_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_200mhz,
							sd_pads_200mhz_cnt);
	} else if (clock > 52000000) {
		if (pad_mode == SD_PAD_MODE_MED_SPEED)
			return 0;
		BUG_ON(!sd_pads_100mhz);
		pad_mode = SD_PAD_MODE_MED_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_100mhz,
							sd_pads_100mhz_cnt);
	} else {
		if (pad_mode == SD_PAD_MODE_LOW_SPEED)
			return 0;
		BUG_ON(!sd_pads_50mhz);
		pad_mode = SD_PAD_MODE_LOW_SPEED;
		return mxc_iomux_v3_setup_multiple_pads(sd_pads_50mhz,
							sd_pads_50mhz_cnt);
	}
}

// (Sylvia) added
static const struct esdhc_platform_data mx6q_sabrelite_sd2_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD2_CD,
	.wp_gpio = -1,//MX6Q_SABRELITE_SD2_WP,
	.support_8bit = 0,
	.keep_power_at_suspend = 1,
	//.platform_pad_change = plt_sd_pad_change,
	.cd_type = ESDHC_CD_GPIO,
	//.delay_line = 0,
};
#if 0
static const struct esdhc_platform_data mx6q_sabrelite_sd3_data __initconst = {
	.cd_gpio = MX6Q_SABRELITE_SD3_CD,
	.wp_gpio = MX6Q_SABRELITE_SD3_WP,
	.keep_power_at_suspend = 1,
	.platform_pad_change = plt_sd_pad_change,
};
#endif

static const struct esdhc_platform_data mx6q_sabrelite_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	// (Sylvia) marked
	//.cd_gpio = -1,//MX6Q_SABRELITE_SD4_CD,
	//.wp_gpio = -1,//MX6Q_SABRELITE_SD4_WP, 
	//.platform_pad_change = plt_sd_pad_change,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

static const struct anatop_thermal_platform_data
	mx6q_sabrelite_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

//(Peter) Fix DTE mode hang issue
static const struct imxuart_platform_data mx6_vab820_uart1_data __initconst = {
        .flags      = IMXUART_HAVE_RTSCTS | IMXUART_USE_DCEDTE | IMXUART_SDMA,
        .dma_req_rx = MX6Q_DMA_REQ_UART1_RX,
        .dma_req_tx = MX6Q_DMA_REQ_UART1_TX,
};

static inline void mx6q_sabrelite_init_uart(void)
{
	imx6q_add_imx_uart(0, &mx6_vab820_uart1_data);
	imx6q_add_imx_uart(1, NULL);
}

// must set 0 or !0
#define FEC_DBG_INFO 0

static void print_ksz9021_dbg_info(struct phy_device *phydev)
{
	int i = 0;
	printk("<1>[IEEE Defined]\n");
	for(i = 0; i <= 0xf; i++){  
		printk("<1> Reg %d: 0x%x", i, phy_read(phydev, i));  
	}  
	printk("<1>\n[Vendor]\n");  

	for(i = 0x11; i <= 0x1f; i++){  
		printk("<1> Reg %d: 0x%x", i, phy_read(phydev, i));  
	}  

	printk("<1>\n[Extended]\n");
	for(i = 256; i <= 261; i++){
		phy_write(phydev, 0x0b, i);
		printk("<1> Reg %d: 0x%x", i, phy_read(phydev, 13));
	}

	printk("<1>================= Peter FEC DBG END ==================\n");
}

static void print_ksz9031_dbg_info(struct phy_device *phydev)
{
	int i = 0;
	printk("<1>[IEEE Defined]\n");
	for(i = 0; i <= 0xf; i++){  
		printk("<1> Reg %d: 0x%x", i, phy_read(phydev, i));  
	}  
	printk("<1>\n[Vendor]\n");  

	for(i = 0x11; i <= 0x1f; i++){  
		printk("<1> Reg %d: 0x%x", i, phy_read(phydev, i));  
	}  

	printk("<1>\n[MMD]\n");  

	//MMD 1h
	phy_write(phydev, 0xd, 0x1);
	phy_write(phydev, 0xe, 0x5a);
	phy_write(phydev, 0xd, 0x4001);
	printk("<1> MMD 1 R 5Ah: 0x%x\n", phy_read(phydev, 0xe));

	//MMD 2h
	phy_write(phydev, 0xd, 0x2);
	phy_write(phydev, 0xe, 0x0);
	phy_write(phydev, 0xd, 0x8002);
	for(i = 0; i <= 0x2b; i++){
		printk("<1> MMD 2 R %xh: 0x%x", i, phy_read(phydev, 0xe));
	}

	//MMD 3h
	phy_write(phydev, 0xd, 0x3);
	phy_write(phydev, 0xe, 0x0);
	phy_write(phydev, 0xd, 0x8003);
	for(i = 0; i <= 0x1; i++){
		printk("<1> MMD 3 R %xh: 0x%x", i, phy_read(phydev, 0xe));
	}

	//MMD 7h
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3c);
	phy_write(phydev, 0xd, 0x4007);
	printk("<1> MMD 7 R 3Ch: 0x%x\n", phy_read(phydev, 0xe));
	phy_write(phydev, 0xd, 0x7);
	phy_write(phydev, 0xe, 0x3d);
	phy_write(phydev, 0xd, 0x4007);
	printk("<1> MMD 7 R 3Dh: 0x%x\n", phy_read(phydev, 0xe));

	//MMD 1Ch
	phy_write(phydev, 0xd, 0x1c);
	phy_write(phydev, 0xe, 0x4);
	phy_write(phydev, 0xd, 0x401c);
	printk("<1> MMD 1C R 4h: 0x%x\n", phy_read(phydev, 0xe));
	phy_write(phydev, 0xd, 0x1c);
	phy_write(phydev, 0xe, 0x23);
	phy_write(phydev, 0xd, 0x401c);
	printk("<1> MMD 1C R 23h: 0x%x\n", phy_read(phydev, 0xe));

	printk("<1>================= Peter FEC DBG END ==================\n");
}

static void init_ksz9031(struct phy_device *phydev)
{
	printk("init_KSZ9031()\n");
	/* min rx data delay */
	if ( 1 ) {
		phy_write(phydev, 0xd, 0x2);
		phy_write(phydev, 0xe, 0x5);
		phy_write(phydev, 0xd, 0x4002);
		phy_write(phydev, 0xe, 0x0);
	}
	// (Sylvia) min tx data delay
	if ( 1 ) {
		phy_write(phydev, 0xd, 0x2);
		phy_write(phydev, 0xe, 0x6);
		phy_write(phydev, 0xd, 0x4002);
		phy_write(phydev, 0xe, 0x0);
	}
	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0xd, 0x2);
	phy_write(phydev, 0xe, 0x4);
	phy_write(phydev, 0xd, 0x4002);
	phy_write(phydev, 0xe, 0x0);

	phy_write(phydev, 0xd, 0x2);
	phy_write(phydev, 0xe, 0x8);
	phy_write(phydev, 0xd, 0x4002);
	phy_write(phydev, 0xe, 0x3ff); //0x1ff);
	//phy_write(phydev, 0xe, 0x3e0); // rx=0
	//phy_write(phydev, 0xe, 0x3e8); // rx=8
	//phy_write(phydev, 0xe, 0x3f0); // rx=0x10
	//phy_write(phydev, 0xe, 0x3f8); // rx=0x18

	if ( FEC_DBG_INFO ) {
		print_ksz9031_dbg_info(phydev);
	}
}

static void init_ksz9021(struct phy_device *phydev)
{
	printk("init_KSZ9021()\n");
	/* min rx data delay */
	phy_write(phydev, 0x0b, 0x8105);
	phy_write(phydev, 0x0c, 0x0000);

	phy_write(phydev, 0x0b, 0x8106);
	phy_write(phydev, 0x0c, 0x0000);

	/* max rx/tx clock delay, min rx/tx control delay */
	phy_write(phydev, 0x0b, 0x8104);
	phy_write(phydev, 0x0c, 0xf0f0);
	phy_write(phydev, 0x0b, 0x104);

	if ( FEC_DBG_INFO ) {
		print_ksz9021_dbg_info(phydev);
	}
}

static int mx6q_sabrelite_fec_phy_init(struct phy_device *phydev)
{
	/* prefer master mode, disable 1000 Base-T capable */
	phy_write(phydev, 0x9, 0x1c00);

	if ( 0x11 == (0xff & phy_read(phydev, 0x03)) ) {
		init_ksz9021(phydev);
	}
	else {
		init_ksz9031(phydev);
	}

	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_sabrelite_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = MX6_ENET_IRQ,
};


static int mx6q_sabrelite_spi_cs1[] = {
	MX6Q_SABRELITE_ECSPI1_CS1,
};

static const struct spi_imx_master mx6q_sabrelite_spi_data1 __initconst = {
	.chipselect     = mx6q_sabrelite_spi_cs1,
	.num_chipselect = ARRAY_SIZE(mx6q_sabrelite_spi_cs1),
};

static int mx6q_sabrelite_spi_cs3[] = {
	MX6Q_SABRELITE_ECSPI3_CS0,
	MX6Q_SABRELITE_ECSPI3_CS1,
};

static const struct spi_imx_master mx6q_sabrelite_spi_data3 __initconst = {
	.chipselect     = mx6q_sabrelite_spi_cs3,
	.num_chipselect = ARRAY_SIZE(mx6q_sabrelite_spi_cs3),
};

#if defined(CONFIG_MTD_M25P80) || defined(CONFIG_MTD_M25P80_MODULE)
/*
static struct mtd_partition imx6_sabrelite_spi_nor_partitions[] = {

	{
	 .name = "bootloader",
	 .offset = 0,
	 .size = 0x00100000,
	},

	{
	 .name = "kernel",
	 .offset = MTDPART_OFS_APPEND,
	 .size = MTDPART_SIZ_FULL,
	},
};
*/
static struct flash_platform_data imx6_sabrelite__spi_flash_data = {
	.name = "m25p80",
//	.parts = imx6_sabrelite_spi_nor_partitions,
//	.nr_parts = ARRAY_SIZE(imx6_sabrelite_spi_nor_partitions),
	.type = "sst25vf032b", //"sst25vf016b",
};


static struct flash_platform_data imx6_sabrelite__spi_flash_micron = {
       .name = "m25p80",
       .type = "w25q32",
};

static struct flash_platform_data imx6_sabrelite__spi_flash_mxic = {
       .name = "m25p80",
       .type = "mx25l3206e",
};

#endif

static struct spi_board_info imx6_sabrelite_spi_nor_device[] __initdata = {
#if defined(CONFIG_MTD_M25P80)
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 0, 	// ECSPI1=0
		.chip_select = 0,
		.platform_data = &imx6_sabrelite__spi_flash_data,
	},
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 2, 	// ECSPI3=2
		.chip_select = 0,
		.platform_data = &imx6_sabrelite__spi_flash_data,
	},
	{
		.modalias = "m25p80",
		.max_speed_hz = 20000000, /* max spi clock (SCK) speed in HZ */
		.bus_num = 2, 	// ECSPI3=2
		.chip_select = 1,
		.platform_data = &imx6_sabrelite__spi_flash_data,
	},


        {
                .modalias = "m25p80",
                .max_speed_hz = 20000000, 
                .bus_num = 0,   // ECSPI1=0
                .chip_select = 0,
                .platform_data = &imx6_sabrelite__spi_flash_micron,
        },
        {
                .modalias = "m25p80",
                .max_speed_hz = 20000000, 
                .bus_num = 0,   // ECSPI1=0
                .chip_select = 0,
                .platform_data = &imx6_sabrelite__spi_flash_mxic,
        },
#endif
};

static void spi_device_init(void)
{
	spi_register_board_info(imx6_sabrelite_spi_nor_device,
				ARRAY_SIZE(imx6_sabrelite_spi_nor_device));
}

static struct mxc_audio_platform_data mx6_sabrelite_audio_data;

static int mx6_sabrelite_sgtl5000_init(void)
{
	struct clk *clko;
	struct clk *new_parent;
	int rate;

	clko = clk_get(NULL, "clko_clk");
	if (IS_ERR(clko)) {
		pr_err("can't get CLKO clock.\n");
		return PTR_ERR(clko);
	}
	new_parent = clk_get(NULL, "ahb");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko, 16000000);
	if (rate < 8000000 || rate > 27000000) {
		pr_err("Error:SGTL5000 mclk freq %d out of range!\n", rate);
		clk_put(clko);
		return -1;
	}

	mx6_sabrelite_audio_data.sysclk = rate;
	clk_set_rate(clko, rate);
	clk_enable(clko);
	return 0;
}

static struct imx_ssi_platform_data mx6_sabrelite_ssi_pdata = {
	.flags = IMX_SSI_DMA | IMX_SSI_SYN,
};

static struct mxc_audio_platform_data mx6_sabrelite_audio_data = {
	.ssi_num = 1,
	.src_port = 2,
	.ext_port = 4,
	.init = mx6_sabrelite_sgtl5000_init,
	.hp_gpio = -1,
};

static struct platform_device mx6_sabrelite_audio_device = {
	.name = "imx-sgtl5000",
};

static struct imxi2c_platform_data mx6q_sabrelite_i2c_data = {
	.bitrate = 100000,
};

#if 0 //(Sylvia) marked
static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};
#endif
#if 0 //(Sylvia) marked
static void mx6q_csi0_cam_powerdown(int powerdown)
{
	if (powerdown)
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 0);

	msleep(2);
}
#endif

static void mx6q_csi0_io_init(void)
{	
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(
			mx6q_sabrelite_csi0_sensor_pads,
			ARRAY_SIZE(mx6q_sabrelite_csi0_sensor_pads));
	} else if (cpu_is_mx6dl()) {	// (Mike@Avnet) added for DL
		mxc_iomux_v3_setup_multiple_pads(
			mx6dl_sabrelite_csi0_sensor_pads,
			ARRAY_SIZE(mx6dl_sabrelite_csi0_sensor_pads));
	}

	/* Camera reset */
	gpio_request(MX6Q_SABRELITE_CSI0_RST, "cam-reset");
	gpio_direction_output(MX6Q_SABRELITE_CSI0_RST, 0);
	msleep(1);
	gpio_set_value(MX6Q_SABRELITE_CSI0_RST, 1);

	/* Camera power down */
	gpio_request(MX6Q_SABRELITE_CSI0_PWN, "cam-pwdn");
	gpio_direction_output(MX6Q_SABRELITE_CSI0_PWN, 1);
	//msleep(1);
	//gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 0);
	//gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 1);

	/* For MX6Q GPR1 bit19 and bit20 meaning:
	 * Bit19:       0 - Enable mipi to IPU1 CSI0
	 *                      virtual channel is fixed to 0
	 *              1 - Enable parallel interface to IPU1 CSI0
	 * Bit20:       0 - Enable mipi to IPU2 CSI1
	 *                      virtual channel is fixed to 3
	 *              1 - Enable parallel interface to IPU2 CSI1
	 * IPU1 CSI1 directly connect to mipi csi2,
	 *      virtual channel is fixed to 1
	 * IPU2 CSI0 directly connect to mipi csi2,
	 *      virtual channel is fixed to 2
	 */

	if (cpu_is_mx6q())
		mxc_iomux_set_gpr_register(1, 19, 1, 1);
	else if (cpu_is_mx6dl()) // (Sylvai) copied from board-mx6q_sabesd.c
		mxc_iomux_set_gpr_register(13, 0, 3, 4);

}

// (Sylvia) copy from board-mx6q_sabreauto.c
static void adv7180_pwdn(int powdn) 
{	// (Sylvia) copied from mx6q_csi0_cam_powerdown()
	if (powdn)
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_CSI0_PWN, 0);

	msleep(2);
}

// (Sylvia) copy from board-mx6q_sabreauto.c
static struct fsl_mxc_tvin_platform_data adv7180_data = {
	.dvddio_reg	= NULL,
	.dvdd_reg	= NULL,
	.avdd_reg	= NULL,
	.pvdd_reg	= NULL,
	.pwdn		= adv7180_pwdn,
	.reset		= NULL,
	.cvbs		= true,
	.io_init	= mx6q_csi0_io_init,
};

#if 0 // (Sylvia) marked
static struct fsl_mxc_camera_platform_data camera_data = {
	.mclk = 24000000,
	.mclk_source = 0,
	.csi = 0,
	.io_init = mx6q_csi0_io_init,
	.pwdn = mx6q_csi0_cam_powerdown,
};
#endif

static struct i2c_board_info mxc_i2c0_board_info[] __initdata = {
	
	{	// (Sylvia) copy from board-mx6q_sabreauto.c
		I2C_BOARD_INFO("adv7180", 0x21),
		.platform_data = (void *)&adv7180_data,
	},

};

static struct i2c_board_info mxc_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
/*	{
		I2C_BOARD_INFO("ov564x", 0x3c),
		.platform_data = (void *)&camera_data,
	},*/
};

static struct i2c_board_info mxc_i2c2_board_info[] __initdata = {
/*	{
		I2C_BOARD_INFO("egalax_ts", 0x4),
		.irq = gpio_to_irq(MX6Q_SABRELITE_CAP_TCH_INT1),
	},*/
	{	// (Sylvia) modified - LVDS
		I2C_BOARD_INFO("egalax_ts", 0x4),
		//.irq = gpio_to_irq(SABRESD_CAP_TCH_INT1),
		.irq = gpio_to_irq(MX6Q_SABRELITE_CAP_TCH_INT0),
	},
	{	// (Sylvia) added
		I2C_BOARD_INFO("sgtl5000", 0x0a),
	},
};

static void imx6q_sabrelite_usbotg_vbus(bool on)
{
	// (Sylvia) USB_OTG_PWR is not used in VAB-820.
	if (on)
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 1);
	else
		gpio_set_value(MX6Q_SABRELITE_USB_OTG_PWR, 0);
}

static void imx6q_sabrelite_usbhost1_vbus(bool on)
{
	if (on)
		gpio_set_value_cansleep(MX6Q_SABRELITE_USB_123_EN, 0);
	else
		gpio_set_value_cansleep(MX6Q_SABRELITE_USB_123_EN, 1);
}

static void __init imx6q_sabrelite_init_usb(void)
{
	int ret = 0;
#if 0
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
	ret = gpio_request(MX6Q_SABRELITE_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO MX6Q_SABRELITE_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_SABRELITE_USB_OTG_PWR, 0);
	mxc_iomux_set_gpr_register(1, 13, 1, 1);

	mx6_set_otghost_vbus_func(imx6q_sabrelite_usbotg_vbus);
#endif
	
	// (Sylvia) added : copied from board-mx6q_sabresd.c
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);
	/* disable external charger detect,
	 * or it will affect signal quality at dp .
	 */
#if 0
	ret = gpio_request(MX6Q_SABRELITE_USB_OTG_PWR, "usb-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_OTG_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_SABRELITE_USB_OTG_PWR, 0);
#endif


#if 1
	/* keep USB host1 VBUS always on */
	ret = gpio_request(MX6Q_SABRELITE_USB_123_EN, "usb-h1-pwr");
	if (ret) {
		pr_err("failed to get GPIO SABRESD_USB_H1_PWR: %d\n",
			ret);
		return;
	}
	gpio_direction_output(MX6Q_SABRELITE_USB_123_EN, 1);
#endif
#if 0
	// (Sylvia) added : setup OC
	ret = gpio_request(MX6Q_SABRELITE_USB_OTG_OC, "otg-oc");
	if (ret) {
		printk(KERN_ERR"failed to get GPIO MX6Q_SABRELITE_USB_OTG_OC:"
			" %d\n", ret);
		return;
	}
	gpio_direction_input(MX6Q_SABRELITE_USB_OTG_OC);

	ret = gpio_request(MX6Q_SABRELITE_USB_123_OC, "usbh1-oc");
	if (ret) {
		printk(KERN_ERR"failed to get MX6Q_SABRELITE_USB_123_OC:"
			" %d\n", ret);
		return;
	}
	gpio_direction_input(MX6Q_SABRELITE_USB_123_OC);

#endif

#if 1
	if (board_is_mx6_reva())
		mxc_iomux_set_gpr_register(1, 13, 1, 1); // base = 0x020E0000  
							 // base + group(1) *4 = 0x020E0004 = IOMUXC_GPR1
							 // start_bits =13 => USB_OTG_ID_SEL
							 // num_bits = 1
							 // value = 1 => selects GPIO_1
	else
		mxc_iomux_set_gpr_register(1, 13, 1, 0);
#endif

	//mx6_set_otghost_vbus_func(imx6q_sabresd_usbotg_vbus); // OTG-PWR-enable decided by OTG ID

	gpio_set_value_cansleep(MX6Q_SABRELITE_USB_123_EN, 0);
	mx6_set_host1_vbus_func(imx6q_sabrelite_usbhost1_vbus);
}

/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_sabrelite_sata_init(struct device *dev, void __iomem *addr)
{
	u32 tmpdata;
	int ret = 0;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFD) | 0x0593A044), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(addr, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(addr + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, addr + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(addr + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_sabrelite_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static struct ahci_platform_data mx6q_sabrelite_sata_data = {
	.init = mx6q_sabrelite_sata_init,
	.exit = mx6q_sabrelite_sata_exit,
};
#endif

/* Ken modified, doesn't need gpio flow control */
static struct gpio mx6q_sabrelite_flexcan_gpios[] = {
        //{ MX6Q_SABRELITE_CAN1_EN, GPIOF_OUT_INIT_LOW, "flexcan1-en" },
        //{ MX6Q_SABRELITE_CAN1_STBY, GPIOF_OUT_INIT_LOW, "flexcan1-stby" },
};

static void mx6q_sabrelite_flexcan0_switch(int enable)
{
        if (enable) {
                //gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 1);
                //gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 1);
        } else {
                //gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 0);
                //gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 0);
        }
}

static void mx6q_sabrelite_flexcan1_switch(int enable)
{
        if (enable) {
                //gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 1);
                //gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 1);
        } else {
                //gpio_set_value(MX6Q_SABRELITE_CAN1_EN, 0);
                //gpio_set_value(MX6Q_SABRELITE_CAN1_STBY, 0);
        }
}


static const struct flexcan_platform_data
        mx6q_sabrelite_flexcan0_pdata __initconst = {
        .transceiver_switch = mx6q_sabrelite_flexcan0_switch,
};

static const struct flexcan_platform_data
        mx6q_sabrelite_flexcan1_pdata __initconst = {
        .transceiver_switch = mx6q_sabrelite_flexcan1_switch,
};

static struct viv_gpu_platform_data imx6q_gpu_pdata __initdata = {
	.reserved_mem_size = SZ_128M,
};

static struct imx_asrc_platform_data imx_asrc_data = {
	.channel_bits = 4,
	.clk_map_ver = 2,
};

/*  Modified by VIA Embedded(SH) 
 *  For RadnR dual display, we need to setup 2 fb devices for display(fb0 and fb2) at least.
 *  Do not care about the values, suhc as disp_dev = "lcd",mode_str = "LDB-XGA", we 
 *  will re-set it in uboot if we want to enable 'hdmi' or 'ldb' or both.
 *  On VAB820, there are only two outputs, hdmi and lvds(dual channel), so fb0 and fb2 are enough.
 */
struct imx_framebuffer_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_framebuffer_mem framebuffer_mem __initdata = {
	.res_msize = SZ_128M,
};

static struct ipuv3_fb_platform_data sabrelite_fb_data[] = {
	{ /*fb0*/
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-XGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB565,
	.mode_str = "CLAA-WVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-SVGA",
	.default_bpp = 16,
	.int_clk = false,
	}, {
	.disp_dev = "lcd",
	.interface_pix_fmt = IPU_PIX_FMT_RGB666,
	.mode_str = "LDB-VGA",
	.default_bpp = 16,
	.int_clk = false,
	},
};

static void hdmi_init(int ipu_id, int disp_id)
{
	int hdmi_mux_setting;

	if ((ipu_id > 1) || (ipu_id < 0)) {
		pr_err("Invalid IPU select for HDMI: %d. Set to 0\n", ipu_id);
		ipu_id = 0;
	}

	if ((disp_id > 1) || (disp_id < 0)) {
		pr_err("Invalid DI select for HDMI: %d. Set to 0\n", disp_id);
		disp_id = 0;
	}

	/* Configure the connection between IPU1/2 and HDMI */
	hdmi_mux_setting = 2*ipu_id + disp_id;

	/* GPR3, bits 2-3 = HDMI_MUX_CTL */
	mxc_iomux_set_gpr_register(3, 2, 2, hdmi_mux_setting);

	/* Set HDMI event as SDMA event2 while Chip version later than TO1.2 */
	
	// (Sylvia) fix the issue HDMI-Audio failed.
	//if ((mx6q_revision() > IMX_CHIP_REVISION_1_1))
	if (hdmi_SDMA_check())	
		mxc_iomux_set_gpr_register(0, 0, 1, 1);
}

/* On mx6x sbarelite board i2c2 iomux with hdmi ddc,
 * the pins default work at i2c2 function,
 when hdcp enable, the pins should work at ddc function */

static void hdmi_enable_ddc_pin(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_hdmi_ddc_pads,
			ARRAY_SIZE(mx6q_sabrelite_hdmi_ddc_pads));
	} else if (cpu_is_mx6dl()) {	// (Mike@Avnet) added for DL
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabrelite_hdmi_ddc_pads,
			ARRAY_SIZE(mx6dl_sabrelite_hdmi_ddc_pads));
	}
}

static void hdmi_disable_ddc_pin(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_i2c2_pads,
			ARRAY_SIZE(mx6q_sabrelite_i2c2_pads));
	} else if (cpu_is_mx6dl()) {	// (Mike@Avnet) added for DL
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabrelite_i2c2_pads,
			ARRAY_SIZE(mx6dl_sabrelite_i2c2_pads));
	}
}

static struct fsl_mxc_hdmi_platform_data hdmi_data = {
	.init = hdmi_init,
	.enable_pins = hdmi_enable_ddc_pin,
	.disable_pins = hdmi_disable_ddc_pin,
};

static struct fsl_mxc_hdmi_core_platform_data hdmi_core_data = {
	.ipu_id = 1,//modifid by roland
	.disp_id = 0,
};

static struct fsl_mxc_lcd_platform_data lcdif_data = {
	.ipu_id = 0,
	.disp_id = 0,
	.default_ifmt = IPU_PIX_FMT_RGB565,
};

static struct fsl_mxc_ldb_platform_data ldb_data = {
	.ipu_id = 1,
	.disp_id = 0,
	.ext_ref = 1,
	.mode = LDB_SEP0,
	.sec_ipu_id = 1,
	.sec_disp_id = 1,
};

static struct imx_ipuv3_platform_data ipu_data[] = {
	{
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	}, {
	.rev = 4,
	.csi_clk[0] = "clko2_clk",
	},
};

static struct fsl_mxc_capture_platform_data capture_data[] = {
	{
		.csi = 0,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 0,
	}, {
		.csi = 1,
		.ipu = 0,
		.mclk_source = 0,
		.is_mipi = 1,
	},
};


struct imx_vout_mem {
	resource_size_t res_mbase;
	resource_size_t res_msize;
};

static struct imx_vout_mem vout_mem __initdata = {
	.res_msize = SZ_128M,
};


static void sabrelite_suspend_enter(void)
{
	/* suspend preparation */
}

static void sabrelite_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_sabrelite_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = sabrelite_suspend_enter,
	.suspend_exit = sabrelite_suspend_exit,
};

#if 0 //defined(CONFIG_KEYBOARD_GPIO) || defined(CONFIG_KEYBOARD_GPIO_MODULE)
#define GPIO_BUTTON(gpio_num, ev_code, act_low, descr, wake)	\
{								\
	.gpio		= gpio_num,				\
	.type		= EV_KEY,				\
	.code		= ev_code,				\
	.active_low	= act_low,				\
	.desc		= "btn " descr,				\
	.wakeup		= wake,					\
}

static struct gpio_keys_button sabrelite_buttons[] = {
	GPIO_BUTTON(MX6Q_SABRELITE_ONOFF_KEY, KEY_POWER, 1, "key-power", 1),
	GPIO_BUTTON(MX6Q_SABRELITE_MENU_KEY, KEY_MENU, 1, "key-memu", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_HOME_KEY, KEY_HOME, 1, "key-home", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_BACK_KEY, KEY_BACK, 1, "key-back", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_VOL_UP_KEY, KEY_VOLUMEUP, 1, "volume-up", 0),
	GPIO_BUTTON(MX6Q_SABRELITE_VOL_DOWN_KEY, KEY_VOLUMEDOWN, 1, "volume-down", 0),
};

static struct gpio_keys_platform_data sabrelite_button_data = {
	.buttons	= sabrelite_buttons,
	.nbuttons	= ARRAY_SIZE(sabrelite_buttons),
};

static struct platform_device sabrelite_button_device = {
	.name		= "gpio-keys",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &sabrelite_button_data,
	}
};

static void __init sabrelite_add_device_buttons(void)
{
	platform_device_register(&sabrelite_button_device);
}
#else
//static void __init sabrelite_add_device_buttons(void) {}
#endif

static struct regulator_consumer_supply sabrelite_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	//REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data sabrelite_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(sabrelite_vmmc_consumers),
	.consumer_supplies = sabrelite_vmmc_consumers,
};

static struct fixed_voltage_config sabrelite_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sabrelite_vmmc_init,
};

static struct platform_device sabrelite_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &sabrelite_vmmc_reg_config,
	},
};

#ifdef CONFIG_SND_SOC_SGTL5000

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vdda = {
	.supply = "VDDA",
	.dev_name = "2-000a", //"0-000a", // (Sylvia) modified for VAB-820, on which sgtl5000 is linked to I2C3
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddio = {
	.supply = "VDDIO",
	.dev_name = "2-000a", //"0-000a",
};

static struct regulator_consumer_supply sgtl5000_sabrelite_consumer_vddd = {
	.supply = "VDDD",
	.dev_name = "2-000a", //"0-000a",
};

static struct regulator_init_data sgtl5000_sabrelite_vdda_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vdda,
};

static struct regulator_init_data sgtl5000_sabrelite_vddio_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddio,
};

static struct regulator_init_data sgtl5000_sabrelite_vddd_reg_initdata = {
	.num_consumer_supplies = 1,
	.consumer_supplies = &sgtl5000_sabrelite_consumer_vddd,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vdda_reg_config = {
	.supply_name		= "VDDA",
	.microvolts		= 3300000, //2500000, (Sylvia) modified for VAB-820 VDDA = 3.3V
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vdda_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddio_reg_config = {
	.supply_name		= "VDDIO",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddio_reg_initdata,
};

static struct fixed_voltage_config sgtl5000_sabrelite_vddd_reg_config = {
	.supply_name		= "VDDD",
	.microvolts		= 0,
	.gpio			= -1,
	.init_data		= &sgtl5000_sabrelite_vddd_reg_initdata,
};

static struct platform_device sgtl5000_sabrelite_vdda_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 0,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vdda_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddio_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 1,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddio_reg_config,
	},
};

static struct platform_device sgtl5000_sabrelite_vddd_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 2,
	.dev	= {
		.platform_data = &sgtl5000_sabrelite_vddd_reg_config,
	},
};

#endif /* CONFIG_SND_SOC_SGTL5000 */

static int imx6q_init_audio(void)
{
	mxc_register_device(&mx6_sabrelite_audio_device,
			    &mx6_sabrelite_audio_data);
	imx6q_add_imx_ssi(1, &mx6_sabrelite_ssi_pdata);
#ifdef CONFIG_SND_SOC_SGTL5000
	platform_device_register(&sgtl5000_sabrelite_vdda_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddio_reg_devices);
	platform_device_register(&sgtl5000_sabrelite_vddd_reg_devices);
#endif
	return 0;
}

static struct platform_pwm_backlight_data mx6_sabrelite_pwm_backlight_data = {
	.pwm_id = 1, //3, (Sylvia) modified for VAB-820
	.max_brightness = 255,
	.dft_brightness = 128,
	.pwm_period_ns = 50000,
};

static struct mxc_dvfs_platform_data sabrelite_dvfscore_data = {
	.reg_id = "cpu_vddgp",
	.soc_id = "cpu_vddsoc",
	.pu_id = "cpu_vddvpu",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};
// steven
//static u32 s_vab820_ram_size = SZ_1G;
u32 s_vab820_ram_size = SZ_1G;
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
 // steven
 struct tag *mem_tag = 0;
 for_each_tag(mem_tag, tags) {
  if (mem_tag->hdr.tag == ATAG_MEM) {
   s_vab820_ram_size = mem_tag->u.mem.size;
   break;
  }
 }
}

#if 1 //enable mipi_csi2 related code to make adv7180 capture workable.
static struct mipi_csi2_platform_data mipi_csi2_pdata = {
	.ipu_id	 = 0,
	.csi_id = 0,
	.v_channel = 0,
	.lanes = 2,
	.dphy_clk = "mipi_pllref_clk",
	.pixel_clk = "emi_clk",
};
#endif

static int __init caam_setup(char *__unused)
{
	caam_enabled = 1;
	return 1;
}
early_param("caam", caam_setup);

static void turnoff_power_led(void)
{
	gpio_request(MX6Q_SABRELITE_GPIO_19_PLED, "power-led");
	gpio_direction_output(MX6Q_SABRELITE_GPIO_19_PLED, 0);
}

//Peter added for power down
static void turnoff_hdmi(void)
{
	void __iomem *mx6_pwr_off = MX6_IO_ADDRESS(0x20c8130);
	u32 value;
	value = readl(mx6_pwr_off);
	writel(value & 0xfffffffe , mx6_pwr_off);

}

static void mx6_poweroff(void)
{
	turnoff_power_led();
	turnoff_hdmi();
}

// (Sylvia) added PCIE
static const struct imx_pcie_platform_data mx6_sabrelite_pcie_data __initconst = {
	.pcie_pwr_en	= MX6Q_SABRELITE_PCIE_CSI_PWN,
	.pcie_rst	= MX6Q_SABRELITE_PCIE_RST_B,
	.pcie_wake_up	= MX6Q_SABRELITE_PCIE_WAKE_B,
	.pcie_dis	= MX6Q_SABRELITE_PCIE_DIS_B,
};

/*!
 * Board specific initialization.
 */
static void __init mx6_sabrelite_board_init(void)
{
	int i;
	int ret;
	struct clk *clko2;
	struct clk *new_parent;
	int rate;
	struct platform_device *voutdev;

	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_sabrelite_pads,
			ARRAY_SIZE(mx6q_sabrelite_pads));
	} else if (cpu_is_mx6dl()) {	// (Mike@Avnet) added for DL
		mxc_iomux_v3_setup_multiple_pads(mx6dl_sabrelite_pads,
			ARRAY_SIZE(mx6dl_sabrelite_pads));
	}
#if 0 
	if (enet_to_gpio_6) {
		iomux_v3_cfg_t enet_gpio_pad =
			MX6Q_PAD_GPIO_6__ENET_IRQ_TO_GPIO_6;
		mxc_iomux_v3_setup_pad(enet_gpio_pad);
	} else {
		/* J5 - Camera GP */
		iomux_v3_cfg_t camera_gpio_pad =
			MX6Q_PAD_GPIO_6__GPIO_1_6;
		mxc_iomux_v3_setup_pad(camera_gpio_pad);
	}
#endif
#if 0 //#ifdef CONFIG_FEC_1588
	/* Set GPIO_16 input for IEEE-1588 ts_clk and RMII reference clock
	 * For MX6 GPR1 bit21 meaning:
	 * Bit21:       0 - GPIO_16 pad output
	 *              1 - GPIO_16 pad input
	 */
	mxc_iomux_set_gpr_register(1, 21, 1, 1);
#endif

	gp_reg_id = sabrelite_dvfscore_data.reg_id;
	soc_reg_id = sabrelite_dvfscore_data.soc_id;
	pu_reg_id = sabrelite_dvfscore_data.pu_id;
	mx6q_sabrelite_init_uart();
	
	if (cpu_is_mx6dl()) {
		hdmi_core_data.ipu_id = 0;
                ldb_data.ipu_id = 0;
                ldb_data.disp_id = 1;
                ldb_data.sec_ipu_id = 0;
                ldb_data.sec_disp_id = 0;
    }
	
	imx6q_add_mxc_hdmi_core(&hdmi_core_data);

	imx6q_add_ipuv3(0, &ipu_data[0]);

	if(s_vab820_ram_size != SZ_1G) {
		sabrelite_fb_data[0].res_base[0] = framebuffer_mem.res_mbase;
		sabrelite_fb_data[0].res_size[0] = framebuffer_mem.res_msize;
	}
	// (Mike@Avnet) modified for DL
	if (cpu_is_mx6q()) {
		imx6q_add_ipuv3(1, &ipu_data[1]);
		for (i = 0; i < 4 && i < ARRAY_SIZE(sabrelite_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabrelite_fb_data[i]);
	} else {
		for (i = 0; i < 2 && i < ARRAY_SIZE(sabrelite_fb_data); i++)
			imx6q_add_ipuv3fb(i, &sabrelite_fb_data[i]);
	}

	imx6q_add_vdoa();
	imx6q_add_lcdif(&lcdif_data);
	imx6q_add_ldb(&ldb_data);
	voutdev = imx6q_add_v4l2_output(0);
	if (vout_mem.res_msize && voutdev) {
		dma_declare_coherent_memory(&voutdev->dev,
					    vout_mem.res_mbase,
					    vout_mem.res_mbase,
					    vout_mem.res_msize,
					    (DMA_MEMORY_MAP |
					     DMA_MEMORY_EXCLUSIVE));
	}


	imx6q_add_v4l2_capture(0, &capture_data[0]);
	//Disable the empty CSI device register to avoid the warning message of "V4L capture slave device not found".
	//imx6q_add_v4l2_capture(1, &capture_data[1]);

	//enable mipi_csi2 related code to make adv7180 capture workable.
	imx6q_add_mipi_csi2(&mipi_csi2_pdata);
	imx6q_add_imx_snvs_rtc();

	//Peter added for power down
	pm_power_off = mx6_poweroff;

	if (1 == caam_enabled)
		imx6q_add_imx_caam();

	imx6q_add_imx_i2c(0, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(1, &mx6q_sabrelite_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_sabrelite_i2c_data);
	i2c_register_board_info(0, mxc_i2c0_board_info,
			ARRAY_SIZE(mxc_i2c0_board_info));
	i2c_register_board_info(1, mxc_i2c1_board_info,
			ARRAY_SIZE(mxc_i2c1_board_info));
	i2c_register_board_info(2, mxc_i2c2_board_info,
			ARRAY_SIZE(mxc_i2c2_board_info));

	/* SPI */
	imx6q_add_ecspi(0, &mx6q_sabrelite_spi_data1);
	imx6q_add_ecspi(2, &mx6q_sabrelite_spi_data3);
	spi_device_init();

	imx6q_add_mxc_hdmi(&hdmi_data);

	imx6q_add_anatop_thermal_imx(1, &mx6q_sabrelite_anatop_thermal_data);
	if (enet_to_gpio_6)
		/* Make sure the IOMUX_OBSRV_MUX1 is set to ENET_IRQ. */
		mxc_iomux_set_specialbits_register(
			IOMUX_OBSRV_MUX1_OFFSET,
			OBSRV_MUX1_ENET_IRQ,
			OBSRV_MUX1_MASK);
	else
		fec_data.gpio_irq = -1;

	imx6_init_fec(fec_data);
	imx6q_add_pm_imx(0, &mx6q_sabrelite_pm_data);
	// (Sylvia) modified
	//imx6q_add_sdhci_usdhc_imx(2, &mx6q_sabrelite_sd3_data); 
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_sabrelite_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_sabrelite_sd2_data);
	imx_add_viv_gpu(&imx6_gpu_data, &imx6q_gpu_pdata);
	imx6q_sabrelite_init_usb();

#if 0	// (Sylvia) marked
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_sabrelite_sata_data);
#else
		mx6q_sabrelite_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
#endif
	imx6q_add_vpu();
	imx6q_init_audio();
	platform_device_register(&sabrelite_vmmc_reg_devices);
	imx_asrc_data.asrc_core_clk = clk_get(NULL, "asrc_clk");
	imx_asrc_data.asrc_audio_clk = clk_get(NULL, "asrc_serial_clk");
	imx6q_add_asrc(&imx_asrc_data);


	/* release USB Hub reset */
	// (Sylvia) modified
	//gpio_set_value(MX6Q_SABRELITE_USB_HUB_RESET, 1);
	//gpio_set_value(MX6Q_SABRELITE_USB_HUB_RST_B, 1);
	gpio_request(MX6Q_SABRELITE_USB_HUB_RST_B, "usbhub-reset");
	gpio_direction_output(MX6Q_SABRELITE_USB_HUB_RST_B, 0);
	msleep(1);
	gpio_set_value(MX6Q_SABRELITE_USB_HUB_RST_B, 1);


	//(Peter) add for UART1 DTR pin
	gpio_request(IMX_GPIO_NR(3, 24), "uart1-dtr");
	gpio_direction_output(IMX_GPIO_NR(3, 24), 0);

	// (Sylvia) modified : VAB-820 uses only PWM2 (MX6Q_PAD_DISP0_DAT9__PWM2_PWMO)
	//imx6q_add_mxc_pwm(0);
	imx6q_add_mxc_pwm(1);
	//imx6q_add_mxc_pwm(2);
	//imx6q_add_mxc_pwm(3);
	imx6q_add_mxc_pwm_backlight(1, &mx6_sabrelite_pwm_backlight_data);
	//imx6q_add_mxc_pwm_backlight(3, &mx6_sabrelite_pwm_backlight_data);

	imx6q_add_otp();
	imx6q_add_viim();
	imx6q_add_imx2_wdt(0, NULL);
	imx6q_add_dma();

	imx6q_add_dvfs_core(&sabrelite_dvfscore_data);

	//(Sylvia) marked
	//sabrelite_add_device_buttons();

	imx6q_add_hdmi_soc();
	imx6q_add_hdmi_soc_dai();

	/* Ken modified */
        ret = gpio_request_array(mx6q_sabrelite_flexcan_gpios,
                        ARRAY_SIZE(mx6q_sabrelite_flexcan_gpios));
        if (ret)
                pr_err("failed to request flexcan1-gpios: %d\n", ret);
        else {
                imx6q_add_flexcan0(&mx6q_sabrelite_flexcan0_pdata);
		imx6q_add_flexcan1(&mx6q_sabrelite_flexcan1_pdata);
	}

	clko2 = clk_get(NULL, "clko2_clk");
	if (IS_ERR(clko2))
		pr_err("can't get CLKO2 clock.\n");

	new_parent = clk_get(NULL, "osc_clk");
	if (!IS_ERR(new_parent)) {
		clk_set_parent(clko2, new_parent);
		clk_put(new_parent);
	}
	rate = clk_round_rate(clko2, 24000000);
	clk_set_rate(clko2, rate);
	clk_enable(clko2);
	imx6q_add_busfreq();

	// (Sylvia) added
	imx6q_add_pcie(&mx6_sabrelite_pcie_data);

	imx6q_add_perfmon(0);
	imx6q_add_perfmon(1);
	imx6q_add_perfmon(2);

	printk("VAB-820......\n");
}

extern void __iomem *twd_base;
static void __init mx6_sabrelite_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART2_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_sabrelite_timer = {
	.init   = mx6_sabrelite_timer_init,
};

static void __init mx6q_sabrelite_reserve(void)
{
	phys_addr_t phys;
	/* steven: for building mfgtool, move 'nSize' to here. */
	u32 nSize = (s_vab820_ram_size==SZ_2G) ? SZ_2G : SZ_1G;
#if defined(CONFIG_MXC_GPU_VIV) || defined(CONFIG_MXC_GPU_VIV_MODULE)

	if (imx6q_gpu_pdata.reserved_mem_size) {
		// steven
		phys = memblock_alloc_base(imx6q_gpu_pdata.reserved_mem_size,
					   SZ_4K, nSize);
		memblock_remove(phys, imx6q_gpu_pdata.reserved_mem_size);
		imx6q_gpu_pdata.reserved_mem_base = phys;
	}
#endif
	if (vout_mem.res_msize) {
		// steven: SZ_1G -> nSize
		phys = memblock_alloc_base(vout_mem.res_msize,
					   SZ_4K, nSize);
		memblock_remove(phys, vout_mem.res_msize);
		vout_mem.res_mbase = phys;
	}


	if ((s_vab820_ram_size != SZ_1G) && framebuffer_mem.res_msize) {
		/*added by via embedded,roland*/
		phys = memblock_alloc_base(framebuffer_mem.res_msize, SZ_4K, nSize);
		memblock_remove(phys, framebuffer_mem.res_msize);
		framebuffer_mem.res_mbase = phys;
	}


}

/*
 * initialize __mach_desc_MX6Q_VAB820 data structure.
 */
MACHINE_START(MX6Q_SABRELITE, "Freescale i.MX 6Quad/DualLite VAB-820 Board")
	/* Maintainer: Freescale Semiconductor, Inc. */
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_sabrelite_board_init,
	.timer = &mx6_sabrelite_timer,
	.reserve = mx6q_sabrelite_reserve,
MACHINE_END
