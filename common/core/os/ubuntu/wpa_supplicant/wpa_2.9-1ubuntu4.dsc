-----BEGIN PGP SIGNED MESSAGE-----
Hash: SHA1

Format: 3.0 (quilt)
Source: wpa
Binary: hostapd, wpagui, wpasupplicant, wpasupplicant-udeb
Architecture: linux-any kfreebsd-any
Version: 2:2.9-1ubuntu4
Maintainer: Ubuntu Developers <ubuntu-devel-discuss@lists.ubuntu.com>
Uploaders:  Andrej Shadura <andrewsh@debian.org>
Homepage: http://w1.fi/wpa_supplicant/
Standards-Version: 4.3.0
Vcs-Browser: https://salsa.debian.org/debian/wpa.git
Vcs-Git: https://salsa.debian.org/debian/wpa.git
Build-Depends: debhelper-compat (= 12), libdbus-1-dev, libssl-dev, qtbase5-dev, libncurses5-dev, libpcsclite-dev, libnl-3-dev (>= 3.4.0~) [linux-any], libnl-genl-3-dev [linux-any], libnl-route-3-dev [linux-any], libpcap-dev [kfreebsd-any], libbsd-dev [kfreebsd-any], libreadline-dev, pkg-config, docbook-to-man, docbook-utils
Package-List:
 hostapd deb net optional arch=linux-any,kfreebsd-any
 wpagui deb net optional arch=linux-any,kfreebsd-any
 wpasupplicant deb net optional arch=linux-any,kfreebsd-any
 wpasupplicant-udeb udeb debian-installer standard arch=linux-any
Checksums-Sha1:
 8c4bafede40b32890ab65ac120e1c24757878248 2347080 wpa_2.9.orig.tar.xz
 ea8512d4b47cc110589667dcc1b713e37ad12bc4 86552 wpa_2.9-1ubuntu4.debian.tar.xz
Checksums-Sha256:
 4032da92d97cb555053d94d514d590d0ce066ca13ba5ef144063450bc56161a7 2347080 wpa_2.9.orig.tar.xz
 007c08b8a8304a94d7cec0f892b4b8b99d0d42c4b226e010d4cd436bbbb97c38 86552 wpa_2.9-1ubuntu4.debian.tar.xz
Files:
 132953a85df36d0fca4df129b036ca06 2347080 wpa_2.9.orig.tar.xz
 481bcdee3ebd9b6e960c810dc44c98f4 86552 wpa_2.9-1ubuntu4.debian.tar.xz
Original-Maintainer: Debian wpasupplicant Maintainers <wpa@packages.debian.org>

-----BEGIN PGP SIGNATURE-----
Version: GnuPG v1

iEYEARECAAYFAl6WwVsACgkQQxo87aLX0pInKQCeOdsVuYwmnUJ1Og4tZV2q5heu
SPwAoOiMAPq8dVQA/I7+PKP32agym/1v
=qdZ5
-----END PGP SIGNATURE-----
