plugin_LTLIBRARIES = libgstmmal.la

libgstmmal_la_SOURCES = \
	gstmmal.c \
	gstmmalmemory.c \
	gstmmalh264dec.c \
	gstmmalmpegviddec.c \
	gstmmalvideodec.c \
	gstmmalvideosink.c

noinst_HEADERS = \
	gstmmalmemory.h \
	gstmmalh264dec.h \
	gstmmalmpegviddec.h \
	gstmmalvideodec.h \
	gstmmalvideosink.h

# N.B. the `-fgnu89-inline` is there to suppress annoying "inline function
# declared but never defined" warning from VC headers.
#
libgstmmal_la_CFLAGS = \
	$(AM_CFLAGS) \
	$(GST_PLUGINS_BASE_CFLAGS) \
	$(GST_BASE_CFLAGS) \
	$(GST_CFLAGS) \
	$(GMODULE_NO_EXPORT_CFLAGS) \
	-fgnu89-inline \
	-Wl,--no-as-needed -Wl,-lmmal -Wl,--as-needed

libgstmmal_la_LIBADD = \
	-lbcm_host \
	-lmmal \
	$(GST_PLUGINS_BASE_LIBS) \
	-lgstaudio-@GST_API_VERSION@ \
	-lgstpbutils-@GST_API_VERSION@ \
	-lgstvideo-@GST_API_VERSION@ \
	$(GST_BASE_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_NO_EXPORT_LIBS)

libgstmmal_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)

EXTRA_DIST =
