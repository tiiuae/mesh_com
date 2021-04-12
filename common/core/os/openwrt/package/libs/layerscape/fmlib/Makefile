#
# Copyright 2020 NXP
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include $(TOPDIR)/rules.mk

PKG_NAME:=fmlib
PKG_VERSION:=LSDK-20.04
PKG_RELEASE:=1

PKG_SOURCE_PROTO:=git
PKG_SOURCE_URL:=https://source.codeaurora.org/external/qoriq/qoriq-components/fmlib
PKG_SOURCE_VERSION:=69a70474cd8411d5a099c34f40760b6567d781d6
PKG_MIRROR_HASH:=872752139199f2273aec00b12f4ed5545203c8b28f72c3f4d20a3317c8c0ea97

PKG_BUILD_PARALLEL:=0

include $(INCLUDE_DIR)/kernel.mk
include $(INCLUDE_DIR)/package.mk

define Package/fmlib
  SECTION:=libs
  CATEGORY:=Libraries
  TITLE:=Layerscape Frame Manager User Space Library
  DEPENDS:=@TARGET_layerscape
endef

define Package/fmlib/description
  The Frame Manager library provides an API on top of the
  Frame Manager driver ioctl calls, that provides a user
  space application with a simple way to configure driver
  parameters and PCD (parse - classify - distribute) rules.
endef

MAKE_FLAGS += \
	all install-libfm-arm \
	KERNEL_SRC="$(LINUX_DIR)" \
	PREFIX="$(PKG_INSTALL_DIR)"

define Build/InstallDev
	$(CP) $(PKG_INSTALL_DIR)/* $(1)/
endef

define Package/fmlib/install
	$(INSTALL_DIR) $(1)/lib
	$(CP) $(PKG_INSTALL_DIR)/lib/* $(1)/lib/
endef

$(eval $(call BuildPackage,fmlib))
