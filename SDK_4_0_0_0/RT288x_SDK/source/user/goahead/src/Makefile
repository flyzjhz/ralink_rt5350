#
# Makefile for the GoAhead web server reference source base
#  for the uClinux OS
#
# Copyright (c) GoAhead Software, Inc. 1995-2000
#
# $Id: Makefile,v 1.69.2.1 2012-02-21 11:35:16 chhung Exp $
#

NAME	= goahead

# User Management switch
UMSW	= -DUSER_MANAGEMENT_SUPPORT
WSC=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_WSC) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_WSC) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_WSC) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_WSC) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_WSC) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_WSC) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_WSC) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_WSC) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_WSC)
WSC_V2=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_WSC_V2) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_WSC_V2) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_WSC_V2) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_WSC_V2) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_WSC_V2) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_WSC_V2) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_WSC_V2) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_WSC_V2) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_WSC_V2)
WDS=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_WDS) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_WDS) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_WDS) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_WDS) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_WDS) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_WDS) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_WDS) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_WDS) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_WDS)
MBSS=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_MBSS) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_MBSS) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_MBSS) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_MBSS) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_MBSS) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_MBSS) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_MBSS) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_MBSS) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_MBSS)
IGMP_SNOOP=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_IGMP_SNOOP) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_IGMP_SNOOP) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_IGMP_SNOOP) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_IGMP_SNOOP) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_IGMP_SNOOP) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_IGMP_SNOOP) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_IGMP_SNOOP) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_IGMP_SNOOP) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_IGMP_SNOOP)
DLS=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_DLS) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_DLS) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_DLS) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_DLS) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_DLS) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_DLS) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_DLS) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_DLS) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_DLS)
DRAF3=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_80211N_DRAFT3) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_80211N_DRAFT3) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_80211N_DRAFT3) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_80211N_DRAFT3) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_80211N_DRAFT3) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_80211N_DRAFT3) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_80211N_DRAFT3) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_80211N_DRAFT3) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_80211N_DRAFT3)
CARRIER=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_CARRIER) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_CARRIER) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_CARRIER) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_CARRIER) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_CARRIER) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_CARRIER) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_CARRIER) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_CARRIER) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_CARRIER)
DFS=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_DFS) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_DFS) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_DFS) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_DFS) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_DFS) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_DFS) 
ABADN=$(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_2850) $(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RALINK_RT2883) $(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RALINK_RT3883) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP)
WAPI = $(CONFIG_RT2860V2_AP_RF_OFFSET)$(CONFIG_RT2860V2_AP_WAPI) $(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_WAPI) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_WAPI) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_WAPI) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_WAPI) $(CONFIG_RT3572_AP_RF_OFFSET)$(CONFIG_RT3572_AP_WAPI) $(CONFIG_RT5572_AP_RF_OFFSET)$(CONFIG_RT5572_AP_WAPI) $(CONFIG_RT3680_iNIC_AP_RF_OFFSET)$(CONFIG_RT3680_iNIC_AP_WAPI) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_AP_WAPI)
APCLI=$(CONFIG_RT2860V2_AP_APCLI)$(CONFIG_RT3090_AP_APCLI)$(CONFIG_RT5392_AP_APCLI)$(CONFIG_RT5592_AP_APCLI)$(CONFIG_RT3593_AP_APCLI)$(CONFIG_RT3572_AP_APCLI)$(CONFIG_RT5572_AP_APCLI)$(CONFIG_RT3680_iNIC_AP_APCLI)$(CONFIG_RTPCI_AP_APCLI)
MULTIPLECARD=$(CONFIG_RT3090_AP_RF_OFFSET)$(CONFIG_RT3090_AP_MULTIPLE_CARD) $(CONFIG_RT5392_AP_RF_OFFSET)$(CONFIG_RT5392_AP_MULTIPLE_CARD) $(CONFIG_RT5592_AP_RF_OFFSET)$(CONFIG_RT5592_AP_MULTIPLE_CARD) $(CONFIG_RT3593_AP_RF_OFFSET)$(CONFIG_RT3593_AP_MULTIPLE_CARD) $(CONFIG_RTPCI_AP_RF_OFFSET)$(CONFIG_RTPCI_MC_SUPPORT)

# Digest Access switch
# DASW	= -DDIGEST_ACCESS_SUPPORT

# SSL switches
ifeq ("$(CONFIG_USER_GOAHEAD_SSL)", "y")
SSLPATCHFILE = matrix_ssl.o sslSocket.o
MATRIXDIR = $(ROOTDIR)/user/matrixssl-1.8.3
SSLINC = $(MATRIXDIR)
SSLLIB = $(MATRIXDIR)/src/libmatrixsslstatic.a
SSLSW = -DWEBS_SSL_SUPPORT -DMATRIX_SSL -I$(SSLINC)
endif

# If-Modified-Support switches (requires math library, libm.a)
# IFMODSW = -DWEBS_IF_MODIFIED_SUPPORT
# IFMODLIB = /usr/lib/libm.a

# Dependencies
DEPEND_FILES	= asp.o balloc.o base64.o cgi.o default.o  \
				  ejlex.o ejparse.o form.o \
				  h.o handler.o mime.o misc.o page.o \
				  ringq.o rom.o \
				  sock.o sockGen.o $(SSLPATCHFILE) \
				  security.o sym.o uemf.o url.o value.o \
				  md5c.o um.o umui.o websda.o emfdb.o \
				  webrom.o webs.o websuemf.o \
				  internet.o utils.o wireless.o \
				  firewall.o management.o $(NAME).o
ifeq ("$(CONFIG_UNIQUE_WPS)", "y")
DEPEND_FILES += wps_unique.o
else
DEPEND_FILES += wps.o
endif
ifneq ("$(CONFIG_RT2860V2_STA)", "")
DEPEND_FILES += station.o
endif
ifneq ("$(CONFIG_RTDEV)", "")
DEPEND_FILES += inic.o
ifneq ("$(CONFIG_UNIQUE_WPS)", "y")
DEPEND_FILES += wps_inic.o
endif
CFLAGS += -DRTDEV_SUPPORT
endif
ifneq ("$(CONFIG_RT2561_AP)", "")
DEPEND_FILES += legacy.o
endif
ifeq ("$(CONFIG_USER_GOAHEAD_IPV6)", "y")
CFLAGS += -DWF_USE_IPV6
endif
ifeq ("$(CONFIG_USER_GOAHEAD_HOSTNAME)", "y")
CFLAGS += -DGA_HOSTNAME_SUPPORT
endif
ifeq ("$(CONFIG_USER_WPA_SUPPLICANT)", "y")
ifeq ("$(CONFIG_RT2860V2_STA_WPA_SUPPLICANT)", "y")
CFLAGS += -DWPA_SUPPLICANT_SUPPORT
endif
endif
ifneq ("$(CONFIG_USB)", "")
DEPEND_FILES += usb.o
endif
ifeq ("$(CONFIG_RALINKAPP_MPLAYER)", "y")
DEPEND_FILES += media.o
endif
ifeq ("$(CONFIG_RALINKAPP_SWQOS)", "y")
DEPEND_FILES += qos.o
endif
ifeq ("$(CONFIG_RALINKAPP_WAPI)", "y")
ifneq ($(findstring 0x40000y, $(WAPI)),)
CFLAGS += -DRT2860_WAPI_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_WAPI_SUPPORT
endif
endif
ifneq ($(findstring 0x48000y, $(WAPI)),)
CFLAGS += -DRTDEV_WAPI_SUPPORT
endif
ifeq ("$(CONFIG_RT2860V2_STA_WAPI)", "y")
CFLAGS += -DSTA_WAPI_SUPPORT
endif
endif

ifneq ($(findstring 0x40000y, $(ABADN)),)
CFLAGS += -DRT2860_ABADN_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_ABADN_SUPPORT
endif
else
ifeq ($(findstring 0x40000m, $(ABADN)),)
CFLAGS += -DRT2860_ABADN_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_ABADN_SUPPORT
endif
endif
endif
ifneq ($(findstring 0x40000y, $(WSC)),)
CFLAGS += -DRT2860_WSC_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_WSC_SUPPORT
endif
ifneq ($(findstring 0x40000y, $(WSC_V2)),)
CFLAGS += -DRT2860_WSCV2_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_WSCV2_SUPPORT
endif
endif
endif
ifneq ($(findstring 0x40000y, $(WDS)),)
CFLAGS += -DRT2860_WDS_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_WDS_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(APCLI)),)
CFLAGS += -DRT2860_APCLI_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_APCLI_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(MBSS)),)
CFLAGS += -DRT2860_MBSS_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_MBSS_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(IGMP_SNOOP)),)
CFLAGS += -DRT2860_IGMPSNOOP_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_IGMPSNOOP_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(DLS)),)
CFLAGS += -DRT2860_DLS_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_DLS_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(DRAF3)),)
CFLAGS += -DRT2860_11NDRAF3_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_11NDRAF3_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(DFS)),)
CFLAGS += -DRT2860_DFS_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_DFS_SUPPORT
endif
endif
ifneq ($(findstring 0x40000y, $(CARRIER)),)
CFLAGS += -DRT2860_CARRIER_SUPPORT
ifneq ($(findstring 0x40000y, $(MULTIPLECARD)),)
CFLAGS += -DRTDEV_CARRIER_SUPPORT
endif
endif

ifneq ($(findstring 0x48000y, $(ABADN)),)
CFLAGS += -DRTDEV_ABADN_SUPPORT
else
ifneq ($(findstring 0x48000m, $(ABADN)),)
CFLAGS += -DRTDEV_ABADN_SUPPORT
endif
endif
ifneq ($(findstring 0x48000y, $(WSC)),)
CFLAGS += -DRTDEV_WSC_SUPPORT
ifneq ($(findstring 0x48000y, $(WSC_V2)),)
CFLAGS += -DRTDEV_WSCV2_SUPPORT
endif
endif
ifneq ($(findstring 0x48000y, $(WDS)),)
CFLAGS += -DRTDEV_WDS_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(MBSS)),)
CFLAGS += -DRTDEV_MBSS_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(IGMP_SNOOP)),)
CFLAGS += -DRTDEV_IGMPSNOOP_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(DLS)),)
CFLAGS += -DRTDEV_DLS_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(DRAF3)),)
CFLAGS += -DRTDEV_11NDRAF3_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(APCLI)),)
CFLAGS += -DRTDEV_APCLI_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(DFS)),)
CFLAGS += -DRTDEV_DFS_SUPPORT
endif
ifneq ($(findstring 0x48000y, $(CARRIER)),)
CFLAGS += -DRTDEV_CARRIER_SUPPORT
endif

CFLAGS	+= -DWEBS -DUEMF -DOS="LINUX" -DLINUX $(UMSW) $(DASW) $(SSLSW) $(IFMODSW)
CFLAGS  += -Wall -fno-strict-aliasing
CFLAGS	+= -I$(ROOTDIR)/lib/libnvram -I$(ROOTDIR)/$(LINUXDIR)/drivers/char -I$(ROOTDIR)/$(LINUXDIR)/include
CFLAGS  += -I$(ROOTDIR)/$(LINUXDIR)/drivers/flash 
OTHERS	= -DB_STATS -DB_FILL -DDEBUG
LDFLAGS	+= $(SSLLIB) $(IFMODLIB)
LDLIBS	+= -lnvram

CONF_H	= $(ROOTDIR)/$(LINUXDIR)/include/linux/autoconf.h
UCONF_H	= $(ROOTDIR)/config/autoconf.h
BUSYCONF_H = $(ROOTDIR)/user/busybox/include/autoconf.h

all:	$(NAME)

#
#	Primary link
#
$(NAME): clean_inet $(DEPEND_FILES)
	$(CC) -o $@ $(DEPEND_FILES) $(LDFLAGS) $(EXTRALIBS) $(LDLIBS)
	$(STRIP) --remove-section=.note --remove-section=.comment $@

romfs:
	$(ROMFSINST) /bin/$(NAME)
ifeq ("$(CONFIG_USER_GOAHEAD_SSL)", "y")
	$(ROMFSINST) /etc_ro/serverkey.pem
	$(ROMFSINST) /etc_ro/servercert.pem
endif

clean:
	rm -f $(NAME) *.o

clean_inet:
	rm -f internet.o

#
#	Dependencies
#
asp.o:  webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

balloc.o: balloc.c uemf.h

base64.o:  base64.c webs.h wsIntrn.h  ej.h ejIntrn.h uemf.h

cgi.o:  webs.h wsIntrn.h uemf.h

default.o:  default.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h $(CONF_H)

ejlex.o:  ejlex.c ej.h ejIntrn.h uemf.h

ejparse.o:  ejparse.c ej.h ejIntrn.h uemf.h

emfdb.o:  emfdb.h wsIntrn.h uemf.h

firewall.o: firewall.c webs.h firewall.h

form.o:  form.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

goahead.o: goahead.c uemf.h wsIntrn.h internet.h utils.h wireless.h firewall.h management.h qos.h $(CONF_H) $(UCONF_H) $(BUSYBOXCONF_H)

h.o:  h.c uemf.h

handler.o:  handler.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

inic.o: inic.c inic.h internet.h utils.h webs.h 

legacy.o: legacy.c legacy.h internet.h utils.h webs.h 

internet.o: internet.c internet.h utils.h webs.h  $(CONF_H) $(UCONF_H) $(BUSYBOXCONF_H)

management.o: management.c management.h webs.h $(CONF_H) $(UCONF_H) 

matrix_ssl.o: matrix_ssl.c wsIntrn.h webs.h websSSL.h sslSocket.h

md5c.o:  md5.h wsIntrn.h uemf.h

media.o: media.c media.h webs.h

mime.o:  mime.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

misc.o:  misc.c uemf.h

page.o:  page.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

ringq.o:  ringq.c uemf.h

rom.o:  rom.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

security.o:  security.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h $(CONF_H)

sock.o:  sock.c uemf.h

sockGen.o:  sockGen.c uemf.h $(CONF_H)

sslSocket.o: sslSocket.c sslSocket.h

station.o: station.c station.h oid.h stapriv.h webs.h $(CONF_H) $(UCONF_H)

usb.o: usb.c usb.h webs.h $(UCONF_H)

sym.o:  sym.c uemf.h

uemf.o:  uemf.c uemf.h

um.o:  webs.h wsIntrn.h um.h uemf.h

umui.o:  webs.h wsIntrn.h um.h uemf.h

url.o:  url.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

utils.o: utils.c utils.h webs.h $(CONF_H) $(UCONF_H)

value.o:  value.c uemf.h

webrom.o:  webrom.c webs.h wsIntrn.h uemf.h

webs.o:  webs.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h $(CONF_H)

websda.o:  webs.h wsIntrn.h websda.h uemf.h

websuemf.o:  websuemf.c webs.h wsIntrn.h ej.h ejIntrn.h uemf.h

websSSL.o:  websSSL.c websSSL.h wsIntrn.h ej.h ejIntrn.h uemf.h

wireless.o: wireless.c wireless.h internet.h utils.h webs.h $(CONF_H) $(UCONF_H)

wps.o: wps.c wps.h utils.h webs.h internet.h wireless.h station.h oid.h $(CONF_H)

wps_inic.o: wps_inic.c wps.h utils.h webs.h internet.h oid.h $(CONF_H)

wps_unique.o: wps_unique.c wps.h utils.h webs.h internet.h oid.h $(CONF_H)

qos.o: qos.h utils.h $(CONF_H)
