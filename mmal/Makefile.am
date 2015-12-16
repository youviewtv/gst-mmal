plugin_LTLIBRARIES = libgstmmal.la

libgstmmal_la_SOURCES = \
	gstmmal.c \
	gstmmalvideosink.c

noinst_HEADERS = \
	gstmmalvideosink.h

libgstmmal_la_CFLAGS = \
	$(GST_PLUGINS_BASE_CFLAGS) \
	$(GST_BASE_CFLAGS) \
	$(GST_CFLAGS) \
	$(GMODULE_NO_EXPORT_CFLAGS)

libgstmmal_la_LIBADD = \
	$(GST_PLUGINS_BASE_LIBS) \
	-lgstaudio-@GST_API_VERSION@ \
	-lgstpbutils-@GST_API_VERSION@ \
	-lgstvideo-@GST_API_VERSION@ \
	$(GST_BASE_LIBS) \
	$(GST_LIBS) \
	$(GMODULE_NO_EXPORT_LIBS)

libgstmmal_la_LDFLAGS = $(GST_PLUGIN_LDFLAGS)

EXTRA_DIST =
