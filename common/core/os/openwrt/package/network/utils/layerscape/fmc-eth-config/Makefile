#
# Copyright 2020 NXP
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=fmc-eth-config
PKG_VERSION:=LSDK-20.04
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://source.codeaurora.org/external/qoriq/qoriq-components/eth-config
PKG_SOURCE_VERSION:=6164664070e45810c793f112781ebcedc979e132
PKG_MIRROR_HASH:=7fa25a793458b8e7f987ccf6ff8a860655d60a81a6d766b6bd1fd1a7d247a5a3

include $(INCLUDE_DIR)/package.mk

define Package/fmc-eth-config
  SECTION:=net
  CATEGORY:=Network
  DEPENDS:=@TARGET_layerscape
  TITLE:=Layerscape Ethernet Configuration Files
endef

define Build/Compile
endef

define Package/fmc-eth-config/install
	$(INSTALL_DIR) $(1)/etc/fmc/config/
	$(CP) $(PKG_BUILD_DIR)/obsolete $(1)/etc/fmc/config/
	$(CP) $(PKG_BUILD_DIR)/private $(1)/etc/fmc/config/
	$(CP) $(PKG_BUILD_DIR)/shared_mac $(1)/etc/fmc/config/
endef

$(eval $(call BuildPackage,fmc-eth-config))
