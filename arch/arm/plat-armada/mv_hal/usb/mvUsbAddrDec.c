/*******************************************************************************
Copyright (C) Marvell International Ltd. and its affiliates

This software file (the "File") is owned and distributed by Marvell
International Ltd. and/or its affiliates ("Marvell") under the following
alternative licensing terms.  Once you have made an election to distribute the
File under one of the following license alternatives, please (i) delete this
introductory statement regarding license alternatives, (ii) delete the two
license alternatives that you have not elected to use and (iii) preserve the
Marvell copyright notice above.

********************************************************************************
Marvell Commercial License Option

If you received this File from Marvell and you have entered into a commercial
license agreement (a "Commercial License") with Marvell, the File is licensed
to you under the terms of the applicable Commercial License.

********************************************************************************
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file or by writing to the Free
Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
********************************************************************************
Marvell BSD License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File under the following licensing terms.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

    *   Redistributions of source code must retain the above copyright notice,
	this list of conditions and the following disclaimer.

    *   Redistributions in binary form must reproduce the above copyright
	notice, this list of conditions and the following disclaimer in the
	documentation and/or other materials provided with the distribution.

    *   Neither the name of Marvell nor the names of its contributors may be
	used to endorse or promote products derived from this software without
	specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*******************************************************************************/

#include "mvCommon.h"
#include "mvOs.h"
#include "ctrlEnv/mvCtrlEnvSpec.h"
#include "mvUsbRegs.h"
#include "mvUsb.h"

MV_TARGET usbAddrDecPrioTab[] = {
#if defined(MV_INCLUDE_SDRAM_CS0)
	SDRAM_CS0,
#endif
#if defined(MV_INCLUDE_SDRAM_CS1)
	SDRAM_CS1,
#endif
#if defined(MV_INCLUDE_SDRAM_CS2)
	SDRAM_CS2,
#endif
#if defined(MV_INCLUDE_SDRAM_CS3)
	SDRAM_CS3,
#endif
#if defined(MV_INCLUDE_CESA) && defined(USB_UNDERRUN_WA)
	CRYPT_ENG,
#endif
#if defined(MV_INCLUDE_PEX)
#if defined(MV_INCLUDE_PEX0)
	PEX0_MEM,
#endif
#if defined(MV_INCLUDE_PEX1)
	PEX1_MEM,
#endif
#endif
	TBL_TERM
};

/*******************************************************************************
* usbWinOverlapDetect - Detect USB address windows overlapping
*
* DESCRIPTION:
*       An unpredicted behaviur is expected in case USB address decode
*       windows overlapps.
*       This function detects USB address decode windows overlapping of a
*       specified window. The function does not check the window itself for
*       overlapping. The function also skipps disabled address decode windows.
*
* INPUT:
*       winNum      - address decode window number.
*       pAddrDecWin - An address decode window struct.
*
* OUTPUT:
*       None.
*
* RETURN:
*       MV_TRUE if the given address window overlap current address
*       decode map, MV_FALSE otherwise, MV_ERROR if reading invalid data
*       from registers.
*
*******************************************************************************/
static MV_STATUS usbWinOverlapDetect(MV_U32 dev, MV_U32 winNum, MV_ADDR_WIN *pAddrWin)
{
	MV_U32 winNumIndex;
	MV_UNIT_WIN_INFO addrDecWin;

	for (winNumIndex = 0; winNumIndex < MV_USB_MAX_ADDR_DECODE_WIN; winNumIndex++) {
		/* Do not check window itself       */
		if (winNumIndex == winNum) {
			continue;
		}

		/* Get window parameters    */
		if (MV_OK != mvUsbWinRead(dev, winNumIndex, &addrDecWin)) {
			mvOsPrintf("%s: ERR. TargetWinGet failed\n", __FUNCTION__);
			return MV_ERROR;
		}

		/* Do not check disabled windows    */
		if (addrDecWin.enable == MV_FALSE)
			continue;

		if (MV_TRUE == mvWinOverlapTest(pAddrWin, &(addrDecWin.addrWin)))
			return MV_TRUE;
	}
	return MV_FALSE;
}

/*******************************************************************************
* mvUsbWinWrite - Set USB target address window
*
* DESCRIPTION:
*       This function sets a peripheral target (e.g. SDRAM bank0, PCI_MEM0)
*       address window, also known as address decode window.
*       After setting this target window, the USB will be able to access the
*       target within the address window.
*
* INPUT:
*       winNum      - USB target address decode window number.
*       pAddrDecWin - USB target window data structure.
*
* OUTPUT:
*       None.
*
* RETURN:
*       MV_ERROR if address window overlapps with other address decode windows.
*       MV_BAD_PARAM if base address is invalid parameter or target is
*       unknown.
*
*******************************************************************************/
MV_STATUS mvUsbWinWrite(MV_U32 dev, MV_U32 winNum, MV_UNIT_WIN_INFO *pDecWin)
{
	MV_U32 sizeReg, baseReg;
	MV_U32 size;

	/* Parameter checking   */
	if (winNum >= MV_USB_MAX_ADDR_DECODE_WIN) {
		mvOsPrintf("%s: ERR. Invalid win num %d\n", __FUNCTION__, winNum);
		return MV_BAD_PARAM;
	}

	/* Check if the requested window overlapps with current windows         */
	if (MV_TRUE == usbWinOverlapDetect(dev, winNum, &pDecWin->addrWin)) {
		mvOsPrintf("%s: ERR. Window %d overlap\n", __FUNCTION__, winNum);
		return MV_ERROR;
	}

	/* check if address is aligned to the size */
	if (MV_IS_NOT_ALIGN(pDecWin->addrWin.baseLow, pDecWin->addrWin.size)) {
		mvOsPrintf("mvUsbWinWrite:Error setting USB window %d.\n"
			   "Address 0x%08x is unaligned to size 0x%x.\n",
			   winNum, pDecWin->addrWin.baseLow, pDecWin->addrWin.size);
		return MV_ERROR;
	}

	if (!MV_IS_POWER_OF_2(pDecWin->addrWin.size)) {
		mvOsPrintf("mvUsbWinWrite: Bad windows size.\n");
		return MV_BAD_PARAM;
	}

	size = pDecWin->addrWin.size / MV_USB_WIN_SIZE_ALIGN - 1;

	/* set Size, Attributes and TargetID */
	sizeReg = (((pDecWin->targetId << MV_USB_WIN_TARGET_OFFSET) & MV_USB_WIN_TARGET_MASK) |
		   ((pDecWin->attrib << MV_USB_WIN_ATTR_OFFSET) & MV_USB_WIN_ATTR_MASK) |
		   ((size << MV_USB_WIN_SIZE_OFFSET) & MV_USB_WIN_SIZE_MASK));

#if defined(MV645xx) || defined(MV646xx)
	/* If window is DRAM with HW cache coherency, make sure bit2 is set */
	sizeReg &= ~MV_USB_WIN_BURST_WR_LIMIT_MASK;

	if ((MV_TARGET_IS_DRAM(pDecWin->target)) && (pDecWin->addrWinAttr.cachePolicy != NO_COHERENCY)) {
		sizeReg |= MV_USB_WIN_BURST_WR_32BIT_LIMIT;
	} else {
		sizeReg |= MV_USB_WIN_BURST_WR_NO_LIMIT;
	}
#endif /* MV645xx || MV646xx */

	if (pDecWin->enable == MV_TRUE)
		sizeReg |= MV_USB_WIN_ENABLE_MASK;
	else
		sizeReg &= ~MV_USB_WIN_ENABLE_MASK;

	/* Update Base value  */
	baseReg = (pDecWin->addrWin.baseLow & MV_USB_WIN_BASE_MASK);

	MV_REG_WRITE(MV_USB_WIN_CTRL_REG(dev, winNum), sizeReg);
	MV_REG_WRITE(MV_USB_WIN_BASE_REG(dev, winNum), baseReg);

	return MV_OK;
}

/*******************************************************************************
* mvUsbWinRead - Get USB peripheral target address window.
*
* DESCRIPTION:
*       Get USB peripheral target address window.
*
* INPUT:
*       winNum - USB target address decode window number.
*
* OUTPUT:
*       pDecWin - USB target window data structure.
*
* RETURN:
*       MV_ERROR if register parameters are invalid.
*
*******************************************************************************/
MV_STATUS mvUsbWinRead(MV_U32 dev, MV_U32 winNum, MV_UNIT_WIN_INFO *pDecWin)
{
	MV_U32 sizeReg, baseReg;

	/* Parameter checking   */
	if (winNum >= MV_USB_MAX_ADDR_DECODE_WIN) {
		mvOsPrintf("%s (dev=%d): ERR. Invalid winNum %d\n", __FUNCTION__, dev, winNum);
		return MV_NOT_SUPPORTED;
	}

	baseReg = MV_REG_READ(MV_USB_WIN_BASE_REG(dev, winNum));
	sizeReg = MV_REG_READ(MV_USB_WIN_CTRL_REG(dev, winNum));

	/* Check if window is enabled   */
	if (sizeReg & MV_USB_WIN_ENABLE_MASK) {
		pDecWin->enable = MV_TRUE;

		/* Extract window parameters from registers */
		pDecWin->targetId = (sizeReg & MV_USB_WIN_TARGET_MASK) >> MV_USB_WIN_TARGET_OFFSET;
		pDecWin->attrib = (sizeReg & MV_USB_WIN_ATTR_MASK) >> MV_USB_WIN_ATTR_OFFSET;
		pDecWin->addrWin.size = (sizeReg & MV_USB_WIN_SIZE_MASK) >> MV_USB_WIN_SIZE_OFFSET;
		pDecWin->addrWin.size = (pDecWin->addrWin.size + 1) * MV_USB_WIN_SIZE_ALIGN;
		pDecWin->addrWin.baseLow = (baseReg & MV_USB_WIN_BASE_MASK);
		pDecWin->addrWin.baseHigh = 0;
	} else {
		pDecWin->enable = MV_FALSE;
	}
	return MV_OK;
}

/*******************************************************************************
* mvUsbWinInit -
*
* INPUT:
*
* OUTPUT:
*
* RETURN:
*       MV_ERROR if register parameters are invalid.
*
*******************************************************************************/
MV_STATUS mvUsbWinInit(MV_U32 dev, MV_UNIT_WIN_INFO *addrWinMap)
{
	MV_UNIT_WIN_INFO *addrDecWin;
	MV_U32 winNum;
	MV_U32 winPrioIndex = 0;

	/* First disable all address decode windows */
	for (winNum = 0; winNum < MV_USB_MAX_ADDR_DECODE_WIN; winNum++) {
		MV_REG_BIT_RESET(MV_USB_WIN_CTRL_REG(dev, winNum), MV_USB_WIN_ENABLE_MASK);
	}

	/* Go through all windows in user table until table terminator          */
	winNum = 0;
	while ((usbAddrDecPrioTab[winPrioIndex] != TBL_TERM) && (winNum < MV_USB_MAX_ADDR_DECODE_WIN)) {
		addrDecWin = &addrWinMap[usbAddrDecPrioTab[winPrioIndex]];
		if (addrDecWin->enable == MV_TRUE) {
#if defined(MV645xx) || defined(MV646xx)
			/* Get the default attributes for that target window */
			mvCtrlDefAttribGet(usbWin.target, &usbWin.addrWinAttr);
#endif /* MV645xx || MV646xx */

			if (MV_OK != mvUsbWinWrite(dev, winNum, addrDecWin))
				return MV_ERROR;
			winNum++;
		}
		winPrioIndex++;
	}
	return MV_OK;
}
