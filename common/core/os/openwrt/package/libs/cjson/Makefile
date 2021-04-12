#
# Copyright 2020 NXP
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=cjson
PKG_VERSION:=v1.7.12
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://github.com/DaveGamble/cJSON.git
PKG_SOURCE_VERSION:=3c8935676a97c7c97bf006db8312875b4f292f6c
PKG_MIRROR_HASH:=22d8fa2eae090f3a06f072319c494112cb61dfa7ccf2c88024e0739d3285815c

CMAKE_BINARY_SUBDIR:=build

include $(INCLUDE_DIR)/package.mk
include $(INCLUDE_DIR)/cmake.mk

define Package/libcjson
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=libcjson
endef

define Package/libcjson/description
  cJSON library
endef

define Build/InstallDev
	$(INSTALL_DIR) $(1)/usr
	$(CP) $(PKG_INSTALL_DIR)/usr/*	$(1)/usr/
endef

define Package/libcjson/install
	$(INSTALL_DIR) $(1)/usr/lib
	$(CP) $(PKG_INSTALL_DIR)/usr/lib/*	$(1)/usr/lib/
endef

$(eval $(call BuildPackage,libcjson))
