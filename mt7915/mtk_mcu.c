#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/inet.h>
#include "mt7915.h"
#include "mcu.h"
#include "mac.h"
#include "testmode.h"

int mt7915_mcu_set_txpower_level(struct mt7915_phy *phy, u8 drop_level)
{
	struct mt7915_dev *dev = phy->dev;
	struct mt7915_sku_val {
		u8 format_id;
		u8 val;
		u8 band;
		u8 _rsv;
	} __packed req = {
		.format_id = 1,
		.band = phy->mt76->band_idx,
		.val = !!drop_level,
	};
	int ret;

	ret = mt76_mcu_send_msg(&dev->mt76,
				MCU_EXT_CMD(TX_POWER_FEATURE_CTRL), &req,
				sizeof(req), true);
	if (ret)
		return ret;

	req.format_id = 2;
	if ((drop_level > 90 && drop_level < 100) || !drop_level)
		req.val = 0;
	else if (drop_level > 60 && drop_level <= 90)
		/* reduce Pwr for 1 dB. */
		req.val = 2;
	else if (drop_level > 30 && drop_level <= 60)
		/* reduce Pwr for 3 dB. */
		req.val = 6;
	else if (drop_level > 15 && drop_level <= 30)
		/* reduce Pwr for 6 dB. */
		req.val = 12;
	else if (drop_level > 9 && drop_level <= 15)
		/* reduce Pwr for 9 dB. */
		req.val = 18;
	else if (drop_level > 0 && drop_level <= 9)
		/* reduce Pwr for 12 dB. */
		req.val = 24;

	return mt76_mcu_send_msg(&dev->mt76,
				 MCU_EXT_CMD(TX_POWER_FEATURE_CTRL), &req,
				 sizeof(req), true);
}

#if defined CONFIG_NL80211_TESTMODE || defined MTK_DEBUG
static void mt7915_txbf_dump_pfmu_tag(struct mt7915_dev *dev, struct mt7915_pfmu_tag *tag)
{
	u32 *raw_t1 = (u32 *)&tag->t1;
	u32 *raw_t2 = (u32 *)&tag->t2;

	dev_info(dev->mt76.dev, "=================== TXBf Profile Tag1 Info ==================\n");
	dev_info(dev->mt76.dev,
		 "DW0 = 0x%08x, DW1 = 0x%08x, DW2 = 0x%08x\n",
		 raw_t1[0], raw_t1[1], raw_t1[2]);
	dev_info(dev->mt76.dev,
		 "DW4 = 0x%08x, DW5 = 0x%08x, DW6 = 0x%08x\n\n",
		 raw_t1[3], raw_t1[4], raw_t1[5]);
	dev_info(dev->mt76.dev, "PFMU ID = %d              Invalid status = %d\n",
		 tag->t1.pfmu_idx, tag->t1.invalid_prof);
	dev_info(dev->mt76.dev, "iBf/eBf = %d\n\n", tag->t1.ebf);
	dev_info(dev->mt76.dev, "DBW   = %d\n", tag->t1.data_bw);
	dev_info(dev->mt76.dev, "SU/MU = %d\n", tag->t1.is_mu);
	dev_info(dev->mt76.dev, "RMSD  = %d\n", tag->t1.rmsd);
	dev_info(dev->mt76.dev,
		 "nrow = %d, ncol = %d, ng = %d, LM = %d, CodeBook = %d MobCalEn = %d\n",
		 tag->t1.nr, tag->t1.nc, tag->t1.ngroup, tag->t1.lm, tag->t1.codebook,
		 tag->t1.mob_cal_en);
	dev_info(dev->mt76.dev, "RU start = %d, RU end = %d\n",
		 tag->t1.ru_start_id, tag->t1.ru_end_id);
	dev_info(dev->mt76.dev, "Mem Col1 = %d, Mem Row1 = %d, Mem Col2 = %d, Mem Row2 = %d\n",
		 tag->t1.col_id1, tag->t1.row_id1, tag->t1.col_id2, tag->t1.row_id2);
	dev_info(dev->mt76.dev, "Mem Col3 = %d, Mem Row3 = %d, Mem Col4 = %d, Mem Row4 = %d\n\n",
		 tag->t1.col_id3, tag->t1.row_id3, tag->t1.col_id4, tag->t1.row_id4);
	dev_info(dev->mt76.dev,
		 "STS0_SNR = 0x%02x, STS1_SNR = 0x%02x, STS2_SNR = 0x%02x, STS3_SNR = 0x%02x\n",
		 tag->t1.snr_sts0, tag->t1.snr_sts1, tag->t1.snr_sts2, tag->t1.snr_sts3);
	dev_info(dev->mt76.dev,
		 "STS4_SNR = 0x%02x, STS5_SNR = 0x%02x, STS6_SNR = 0x%02x, STS7_SNR = 0x%02x\n",
		 tag->t1.snr_sts4, tag->t1.snr_sts5, tag->t1.snr_sts6, tag->t1.snr_sts7);
	dev_info(dev->mt76.dev, "=============================================================\n");

	dev_info(dev->mt76.dev, "=================== TXBf Profile Tag2 Info ==================\n");
	dev_info(dev->mt76.dev,
		 "DW0 = 0x%08x, DW1 = 0x%08x, DW2 = 0x%08x\n",
		 raw_t2[0], raw_t2[1], raw_t2[2]);
	dev_info(dev->mt76.dev,
		 "DW3 = 0x%08x, DW4 = 0x%08x, DW5 = 0x%08x\n\n",
		 raw_t2[3], raw_t2[4], raw_t2[5]);
	dev_info(dev->mt76.dev, "Smart antenna ID = 0x%x,  SE index = %d\n",
		 tag->t2.smart_ant, tag->t2.se_idx);
	dev_info(dev->mt76.dev, "RMSD threshold = %d\n", tag->t2.rmsd_thres);
	dev_info(dev->mt76.dev, "Timeout = 0x%x\n", tag->t2.ibf_timeout);
	dev_info(dev->mt76.dev, "Desired BW = %d, Desired Ncol = %d, Desired Nrow = %d\n",
		 tag->t2.ibf_data_bw, tag->t2.ibf_nc, tag->t2.ibf_nr);
	dev_info(dev->mt76.dev, "Desired RU Allocation = %d\n", tag->t2.ibf_ru);
	dev_info(dev->mt76.dev, "Mobility DeltaT = %d, Mobility LQ = %d\n",
		 tag->t2.mob_delta_t, tag->t2.mob_lq_result);
	dev_info(dev->mt76.dev, "=============================================================\n");
}

static void mt7915_txbf_dump_sta_rec(struct mt7915_dev *dev, struct sta_rec_bf *sta_info)
{
	dev_info(dev->mt76.dev, "===================== BF Station Record =====================\n");
	dev_info(dev->mt76.dev, "pfmu           = %d\n", sta_info->pfmu);
	dev_info(dev->mt76.dev, "su_mu          = %d\n", sta_info->su_mu);
	dev_info(dev->mt76.dev, "bf_cap         = %d\n", sta_info->bf_cap);
	dev_info(dev->mt76.dev, "sounding_phy   = %d\n", sta_info->sounding_phy);
	dev_info(dev->mt76.dev, "ndpa_rate      = %d\n", sta_info->ndpa_rate);
	dev_info(dev->mt76.dev, "ndp_rate       = %d\n", sta_info->ndp_rate);
	dev_info(dev->mt76.dev, "rept_poll_rate = %d\n", sta_info->rept_poll_rate);
	dev_info(dev->mt76.dev, "tx_mode        = %d\n", sta_info->tx_mode);
	dev_info(dev->mt76.dev, "ncol           = %d\n", sta_info->ncol);
	dev_info(dev->mt76.dev, "nrow           = %d\n", sta_info->nrow);
	dev_info(dev->mt76.dev, "bw             = %d\n", sta_info->bw);
	dev_info(dev->mt76.dev, "mem_total      = %d\n", sta_info->mem_total);
	dev_info(dev->mt76.dev, "mem_20m        = %d\n", sta_info->mem_20m);
	dev_info(dev->mt76.dev, "mem_row0       = %d\n", sta_info->mem[0].row);
	dev_info(dev->mt76.dev, "mem_col0       = %d\n", sta_info->mem[0].col);
	dev_info(dev->mt76.dev, "mem_row1       = %d\n", sta_info->mem[1].row);
	dev_info(dev->mt76.dev, "mem_col1       = %d\n", sta_info->mem[1].col);
	dev_info(dev->mt76.dev, "mem_row2       = %d\n", sta_info->mem[2].row);
	dev_info(dev->mt76.dev, "mem_col2       = %d\n", sta_info->mem[2].col);
	dev_info(dev->mt76.dev, "mem_row3       = %d\n", sta_info->mem[3].row);
	dev_info(dev->mt76.dev, "mem_col3       = %d\n", sta_info->mem[3].col);
	dev_info(dev->mt76.dev, "smart_ant      = 0x%x\n", sta_info->smart_ant);
	dev_info(dev->mt76.dev, "se_idx         = %d\n", sta_info->se_idx);
	dev_info(dev->mt76.dev, "auto_sounding  = %d\n", sta_info->auto_sounding);
	dev_info(dev->mt76.dev, "ibf_timeout    = 0x%x\n", sta_info->ibf_timeout);
	dev_info(dev->mt76.dev, "ibf_dbw        = %d\n", sta_info->ibf_dbw);
	dev_info(dev->mt76.dev, "ibf_ncol       = %d\n", sta_info->ibf_ncol);
	dev_info(dev->mt76.dev, "ibf_nrow       = %d\n", sta_info->ibf_nrow);
	dev_info(dev->mt76.dev, "nrow_gt_bw80   = %d\n", sta_info->nrow_gt_bw80);
	dev_info(dev->mt76.dev, "ncol_gt_bw80   = %d\n", sta_info->ncol_gt_bw80);
	dev_info(dev->mt76.dev, "ru_start_idx   = %d\n", sta_info->ru_start_idx);
	dev_info(dev->mt76.dev, "trigger_su     = %d\n", sta_info->trigger_su);
	dev_info(dev->mt76.dev, "trigger_mu     = %d\n", sta_info->trigger_mu);
	dev_info(dev->mt76.dev, "ng16_su        = %d\n", sta_info->ng16_su);
	dev_info(dev->mt76.dev, "ng16_mu        = %d\n", sta_info->ng16_mu);
	dev_info(dev->mt76.dev, "codebook42_su  = %d\n", sta_info->codebook42_su);
	dev_info(dev->mt76.dev, "codebook75_mu  = %d\n", sta_info->codebook75_mu);
	dev_info(dev->mt76.dev, "he_ltf         = %d\n", sta_info->he_ltf);
	dev_info(dev->mt76.dev, "=============================================================\n");
}

static void mt7915_txbf_dump_cal_phase(struct mt7915_dev *dev,
				       struct mt7915_txbf_phase *phase, int group)
{
	dev_info(dev->mt76.dev, "Group %d and Group M\n", group);
	dev_info(dev->mt76.dev, "m_t0_h = %d\n", phase->phase.m_t0_h);
	dev_info(dev->mt76.dev, "m_t1_h = %d\n", phase->phase.m_t1_h);
	dev_info(dev->mt76.dev, "m_t2_h = %d\n", phase->phase.m_t2_h);

	dev_info(dev->mt76.dev, "r0_uh = %d\n", phase->phase.r0_uh);
	dev_info(dev->mt76.dev, "r0_h = %d\n", phase->phase.r0_h);
	dev_info(dev->mt76.dev, "r0_m = %d\n", phase->phase.r0_m);
	dev_info(dev->mt76.dev, "r0_l = %d\n", phase->phase.r0_l);

	dev_info(dev->mt76.dev, "r1_uh = %d\n", phase->phase.r1_uh);
	dev_info(dev->mt76.dev, "r1_h = %d\n", phase->phase.r1_h);
	dev_info(dev->mt76.dev, "r1_m = %d\n", phase->phase.r1_m);
	dev_info(dev->mt76.dev, "r1_l = %d\n", phase->phase.r1_l);

	dev_info(dev->mt76.dev, "r2_uh = %d\n", phase->phase.r2_uh);
	dev_info(dev->mt76.dev, "r2_h = %d\n", phase->phase.r2_h);
	dev_info(dev->mt76.dev, "r2_m = %d\n", phase->phase.r2_m);
	dev_info(dev->mt76.dev, "r2_l = %d\n", phase->phase.r2_l);

	dev_info(dev->mt76.dev, "r3_uh = %d\n", phase->phase.r3_uh);
	dev_info(dev->mt76.dev, "r3_h = %d\n", phase->phase.r3_h);
	dev_info(dev->mt76.dev, "r3_m = %d\n", phase->phase.r3_m);
	dev_info(dev->mt76.dev, "r3_l = %d\n", phase->phase.r3_l);
	dev_info(dev->mt76.dev, "r3_ul = %d\n", phase->phase.r3_ul);
}

int mt7915_mcu_txbf_status_read(struct mt7915_dev *dev, struct sk_buff *skb)
{
#define BF_PFMU_TAG	16
#define BF_STA_REC	20
#define BF_CAL_PHASE	21
#define GROUP_M		1
	u8 format_id;

	skb_pull(skb, sizeof(struct mt76_connac2_mcu_rxd));
	format_id = *(u8 *)skb->data;

	if (format_id == BF_PFMU_TAG) {
		struct mt7915_pfmu_tag *pfmu_tag;

		skb_pull(skb, 8);
		pfmu_tag = (struct mt7915_pfmu_tag *)skb->data;
		mt7915_txbf_dump_pfmu_tag(dev, pfmu_tag);
		if (dev->test.txbf_pfmu_tag)
			memcpy(dev->test.txbf_pfmu_tag, pfmu_tag, sizeof(struct mt7915_pfmu_tag));
	} else if (format_id == BF_STA_REC) {
		struct sta_rec_bf *sta_rec;

		skb_pull(skb, sizeof(struct mt7915_bf_status_hdr));
		/* padding 4 byte since bf_status->buf does not contain tag & len */
		skb_push(skb, 4);
		sta_rec = (struct sta_rec_bf *)skb->data;

		mt7915_txbf_dump_sta_rec(dev, sta_rec);
	} else if (format_id == BF_CAL_PHASE) {
		u8 phase_out_len = sizeof(struct mt7915_txbf_phase_out);
		struct mt7915_ibf_cal_info *cal;
		struct mt7915_txbf_phase_out phase_out;
		struct mt7915_txbf_phase *phase =
			(struct mt7915_txbf_phase *)dev->test.txbf_phase_cal;

		cal = (struct mt7915_ibf_cal_info *)skb->data;
		memcpy(&phase_out, cal->buf, phase_out_len);
		switch (cal->cal_type) {
		case IBF_PHASE_CAL_NORMAL:
		case IBF_PHASE_CAL_NORMAL_INSTRUMENT:
			/* Only calibrate group M */
			if (cal->group_l_m_n != GROUP_M)
				break;
			phase = &phase[cal->group];
			memcpy(&phase->phase, cal->buf + phase_out_len, sizeof(phase->phase));
			phase->status = cal->status;

			dev_info(dev->mt76.dev, "Calibrated result = %d\n", phase->status);
			mt7915_txbf_dump_cal_phase(dev, phase, cal->group);
			break;
		case IBF_PHASE_CAL_VERIFY:
		case IBF_PHASE_CAL_VERIFY_INSTRUMENT:
			dev_info(dev->mt76.dev, "Verification result = %d\n", cal->status);
			break;
		default:
			break;
		}

		dev_info(dev->mt76.dev, "c0_h = %d, c1_h = %d, c2_h = %d\n",
			 phase_out.c0_h, phase_out.c1_h, phase_out.c2_h);
		dev_info(dev->mt76.dev, "c0_m = %d, c1_m = %d, c2_m = %d\n",
			 phase_out.c0_m, phase_out.c1_m, phase_out.c2_m);
		dev_info(dev->mt76.dev, "c0_l = %d, c1_l = %d, c2_l = %d\n",
			 phase_out.c0_l, phase_out.c1_l, phase_out.c2_l);
		dev_info(dev->mt76.dev, "c3_m = %d, c3_h = %d\n", phase_out.c3_m, phase_out.c3_h);
	}

	wake_up(&dev->mt76.tx_wait);

	return 0;
}

int mt7915_mcu_txbf_profile_tag_read(struct mt7915_phy *phy, u8 pfmu_idx, bool bfer)
{
	struct mt7915_dev *dev = phy->dev;
	struct {
		u8 format_id;
		u8 pfmu_idx;
		bool bfer;
		u8 dbdc_idx;
	} __packed req = {
		.format_id = MT_BF_PFMU_TAG_READ,
		.pfmu_idx = pfmu_idx,
		.bfer = bfer,
		.dbdc_idx = phy->mt76->band_idx,
	};
	struct mt7915_pfmu_tag *tag = dev->test.txbf_pfmu_tag;

	/* Reset to 0 for mt7915_tm_txbf_profile_tag_write wait_event */
	if (tag)
		tag->t1.pfmu_idx = 0;

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(TXBF_ACTION), &req,
				 sizeof(req), true);
}

int mt7915_mcu_txbf_sta_rec_read(struct mt7915_dev *dev, u16 wlan_idx)
{
	struct {
		u8 action;
		u8 wlan_idx_lo;
		u8 wlan_idx_hi;
		u8 rsv[5];
	} __packed req = {
		.action = MT_BF_STA_REC_READ,
		.wlan_idx_lo = to_wcid_lo(wlan_idx),
		.wlan_idx_hi = to_wcid_hi(wlan_idx),
	};

	return mt76_mcu_send_msg(&dev->mt76, MCU_EXT_CMD(TXBF_ACTION), &req,
				 sizeof(req), true);
}
#endif
