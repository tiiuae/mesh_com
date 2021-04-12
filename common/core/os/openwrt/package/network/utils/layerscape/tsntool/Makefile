#
# Copyright 2020 NXP
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=tsntool
PKG_VERSION:=LSDK-20.04
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://source.codeaurora.org/external/qoriq/qoriq-components/tsntool
PKG_SOURCE_VERSION:=ca2d8fb348bb54960d706177108c43ae213e0063
PKG_MIRROR_HASH:=5e23dc54994ac85fc7de0f3a18e0743639578f4c590e44a4c0684089c77c978d

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define Package/tsntool
  SECTION:=net
  CATEGORY:=Network
  TITLE:=Layerscape TSN tool
  DEPENDS:=@TARGET_layerscape +libnl +libnl-genl +libreadline +libncurses +libcjson
endef

define Build/Compile
	cd $(PKG_BUILD_DIR); \
	mkdir -p include/linux/; \
	cp $(LINUX_DIR)/include/uapi/linux/tsn.h include/linux/; \
	cp $(LINUX_DIR)/include/uapi/linux/net_tstamp.h include/linux/; \
	sed -i 's/termcap/ncurses/g' Makefile; \
	export CC=$(TARGET_CROSS)gcc; \
	$(MAKE)
endef

define Package/tsntool/install
	$(INSTALL_DIR) $(1)/usr/bin/
	$(CP) $(PKG_BUILD_DIR)/tsntool $(1)/usr/bin/
	$(INSTALL_DIR) $(1)/usr/lib/
	$(CP) $(PKG_BUILD_DIR)/libtsn.so $(1)/usr/lib/
endef

$(eval $(call BuildPackage,tsntool))
