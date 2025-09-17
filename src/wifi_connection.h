#ifndef __WIFI_CONNECTION_H
#define __WIFI_CONNECTION_H

class WifiConnection
{
public:
    static void connect(const char *ssid, const char *password);
    static bool connected;

    WifiConnection();
    WifiConnection(WifiConnection const &); // Don't Implement
    void operator=(WifiConnection const &); // Don't implement
};

#endif // __WIFI_CONNECTION_H
