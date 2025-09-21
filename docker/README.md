```
diff --git a/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig b/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig
index f0a8d6ae86..efc516682e 100644
--- a/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig
+++ b/boards/px4/fmu-v6x/nuttx-config/nsh/defconfig
@@ -84,6 +84,8 @@ CONFIG_CDCACM_RXBUFSIZE=600
 CONFIG_CDCACM_TXBUFSIZE=12000
 CONFIG_CDCACM_VENDORID=0x3185
 CONFIG_CDCACM_VENDORSTR="Auterion"
+CONFIG_CRYPTO=y
+CONFIG_CRYPTO_RANDOM_POOL=y
 CONFIG_DEBUG_FULLOPT=y
 CONFIG_DEBUG_HARDFAULT_ALERT=y
 CONFIG_DEBUG_MEMFAULT=y
```

```
cp encrypted_logs.px4board ../boards/px4/fmu-v6x/encrypted_logs.px4board
```
