PREFIX?= /usr/local
BINDIR?= /usr/local/sbin
PROG= martin-usb-dmx
SRCS= martin-usb-dmx.c
MAN=
CFLAGS= -I${PREFIX}/include -Wno-trigraphs
LDFLAGS= -lusb -lpthread -L${PREFIX}/lib -lasound

.if defined(HAVE_PICTURE)
CFLAGS += -DHAVE_PICTURE
.endif

.include <bsd.prog.mk>
