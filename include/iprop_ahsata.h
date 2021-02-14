/*
 * Copyright 2017 Google, Inc
 * Written by Simon Glass <sjg@chromium.org>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __IPROP_AHSATA_H__
#define __IPROP_AHSATA_H__

int iprop_ahsata_bus_reset(struct udevice *dev);
int iprop_ahsata_probe(struct udevice *dev);
int iprop_ahsata_scan(struct udevice *dev);
int iprop_ahsata_port_status(struct udevice *dev, int port);

#endif
