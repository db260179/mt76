// SPDX-License-Identifier: ISC
#include "mt7915.h"
#include "vendor.h"

static const struct nla_policy
edcca_ctrl_policy[NUM_MTK_VENDOR_ATTRS_EDCCA_CTRL] = {
	[MTK_VENDOR_ATTR_EDCCA_CTRL_MODE] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_PRI20_VAL] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC20_VAL] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC40_VAL] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC80_VAL] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC160_VAL] = { .type = NLA_U8 },
	[MTK_VENDOR_ATTR_EDCCA_CTRL_COMPENSATE] = { .type = NLA_S8 },
};

static int mt7915_vendor_edcca_ctrl(struct wiphy *wiphy,
				  struct wireless_dev *wdev,
				  const void *data,
				  int data_len)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct nlattr *tb[NUM_MTK_VENDOR_ATTRS_EDCCA_CTRL];
	int err;
	u8 edcca_mode;
	s8 edcca_compensation;
	u8 edcca_value[EDCCA_THRES_NUM] = {0};

	err = nla_parse(tb, MTK_VENDOR_ATTR_EDCCA_CTRL_MAX, data, data_len,
			edcca_ctrl_policy, NULL);
	if (err)
		return err;

	if (!tb[MTK_VENDOR_ATTR_EDCCA_CTRL_MODE])
		return -EINVAL;

	edcca_mode = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_MODE]);
	if (edcca_mode == EDCCA_CTRL_SET_EN) {
		if (!tb[MTK_VENDOR_ATTR_EDCCA_CTRL_PRI20_VAL] ||
			!tb[MTK_VENDOR_ATTR_EDCCA_CTRL_COMPENSATE]) {
			return -EINVAL;
		}
		edcca_value[0] = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_PRI20_VAL]);
		edcca_compensation = nla_get_s8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_COMPENSATE]);

		err = mt7915_mcu_set_edcca(phy, edcca_mode, edcca_value, edcca_compensation);
		if (err)
			return err;
	} else if (edcca_mode == EDCCA_CTRL_SET_THERS) {
		if (!tb[MTK_VENDOR_ATTR_EDCCA_CTRL_PRI20_VAL] ||
		    !tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC40_VAL] ||
		    !tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC80_VAL] ||
		    !tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC160_VAL]) {
			return -EINVAL;
		}
		edcca_value[0] = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_PRI20_VAL]);
		edcca_value[1] = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC40_VAL]);
		edcca_value[2] = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC80_VAL]);
		edcca_value[3] = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_SEC160_VAL]);
		edcca_compensation = 0;
		if (tb[MTK_VENDOR_ATTR_EDCCA_CTRL_COMPENSATE])
			edcca_compensation = nla_get_s8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_COMPENSATE]);
		err = mt7915_mcu_set_edcca(phy, edcca_mode, edcca_value, edcca_compensation);
		if (err)
			return err;
	} else {
		return -EINVAL;
	}

	return 0;
}

static int
mt7915_vendor_edcca_ctrl_dump(struct wiphy *wiphy, struct wireless_dev *wdev,
			     struct sk_buff *skb, const void *data, int data_len,
			     unsigned long *storage)
{
	struct ieee80211_hw *hw = wiphy_to_ieee80211_hw(wiphy);
	struct mt7915_phy *phy = mt7915_hw_phy(hw);
	struct nlattr *tb[NUM_MTK_VENDOR_ATTRS_EDCCA_CTRL];
	int len = EDCCA_THRES_NUM;
	int err;
	u8 edcca_mode;
	s8 value[EDCCA_THRES_NUM];

	if (*storage == 1)
		return -ENOENT;
	*storage = 1;

	err = nla_parse(tb, MTK_VENDOR_ATTR_EDCCA_CTRL_MAX, data, data_len,
			edcca_ctrl_policy, NULL);
	if (err)
		return err;

	if (!tb[MTK_VENDOR_ATTR_EDCCA_CTRL_MODE])
		return -EINVAL;

	edcca_mode = nla_get_u8(tb[MTK_VENDOR_ATTR_EDCCA_CTRL_MODE]);
	if (edcca_mode == EDCCA_CTRL_GET_EN || edcca_mode == EDCCA_CTRL_GET_THERS) {
		err = mt7915_mcu_get_edcca(phy, edcca_mode, value);
	} else {
		return -EINVAL;
	}

	if (err)
		return err;

	if (nla_put_u8(skb, MTK_VENDOR_ATTR_EDCCA_DUMP_PRI20_VAL, value[0]) ||
	    nla_put_u8(skb, MTK_VENDOR_ATTR_EDCCA_DUMP_SEC40_VAL, value[1]) ||
	    nla_put_u8(skb, MTK_VENDOR_ATTR_EDCCA_DUMP_SEC80_VAL, value[2]) ||
	    nla_put_u8(skb, MTK_VENDOR_ATTR_EDCCA_DUMP_SEC160_VAL, value[3]))
		return -ENOMEM;

	return len;
}

static const struct wiphy_vendor_command mt7915_vendor_commands[] = {
	{
		.info = {
			.vendor_id = MTK_NL80211_VENDOR_ID,
			.subcmd = MTK_NL80211_VENDOR_SUBCMD_EDCCA_CTRL,
		},
		.flags = WIPHY_VENDOR_CMD_NEED_NETDEV |
			 WIPHY_VENDOR_CMD_NEED_RUNNING,
		.doit = mt7915_vendor_edcca_ctrl,
		.dumpit = mt7915_vendor_edcca_ctrl_dump,
		.policy = edcca_ctrl_policy,
		.maxattr = MTK_VENDOR_ATTR_EDCCA_CTRL_MAX,
	},
};

void mt7915_register_vendor(struct mt7915_phy *phy)
{
	phy->mt76->hw->wiphy->vendor_commands = mt7915_vendor_commands;
	phy->mt76->hw->wiphy->n_vendor_commands = ARRAY_SIZE(mt7915_vendor_commands);
}
