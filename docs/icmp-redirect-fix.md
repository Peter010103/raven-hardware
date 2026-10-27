# Pi Internet access: ICMP redirect fix

Raven's Pi bring-up on **24 October 2026** exposed a routing fault where **ping worked but APT, HTTP and HTTPS stalled**. This guide preserves the observed cause, persistent fix and verification commands. The historical fix was checked after reboot; this document does not imply a new live network test.

## Symptoms and observed cause

APT initially appeared stuck connecting to an IPv6 address. IPv4 HTTP and HTTPS also stalled, so forcing IPv4 did not resolve the underlying fault. The Pi's local firewall was inactive with ACCEPT policies.

Packet capture showed the configured robotlab gateway **10.0.0.1** sending **ICMP redirects advertising 10.100.100.2** as an alternative next hop. The Pi accepted and cached that route:

1. The initial TCP SYN used 10.0.0.1 and received a SYN-ACK.
2. Subsequent ACK and HTTP/TLS request packets followed the redirected next hop.
3. Those packets repeatedly retransmitted and the connection stalled.

ICMP echo success alone therefore did not establish a working TCP path. The fix makes the Pi ignore IPv4 redirects and continue using its configured gateway.

## Recorded network layout

| Pi connection | Pi address | Configured gateway |
|---|---|---|
| Original robotlab Wi-Fi / retained fallback | 10.0.0.201/24 | 10.0.0.1 |
| Later robomaster 5 GHz Wi-Fi | 10.3.1.201/24 | 10.3.1.1 |
| Direct Ethernet to desktop | 192.168.77.2/24 | None |

The desktop's direct-cable address is 192.168.77.1/24; its recorded robotlab address is 10.0.0.200/24. DNS for the Pi's lab Wi-Fi profiles is 10.0.0.1. robotlab and robomaster are separate routed subnets. The observed redirect fault was diagnosed on the original robotlab connection; the later robomaster gateway is different.

## Persistent fix on the Pi

The recorded fix is stored in `/etc/sysctl.d/90-raven-network.conf`:

```sh
sudo tee /etc/sysctl.d/90-raven-network.conf >/dev/null <<'CONF'
net.ipv4.conf.all.accept_redirects = 0
net.ipv4.conf.default.accept_redirects = 0
net.ipv4.conf.wlan0.accept_redirects = 0
net.ipv4.conf.eth0.accept_redirects = 0
CONF

sudo sysctl --system
sudo ip route flush cache
```

Run these commands **on the Pi**. If that file already contains other custom settings, preserve them when adding the four entries. Use the actual interface names if the image names them differently.

Both global and per-interface settings are intentional. In Linux host mode, redirect acceptance can remain enabled if either `conf/all` or the interface's `accept_redirects` is enabled. Setting `default` also supplies the policy for interfaces created later. See the [Linux kernel IP sysctl documentation](https://docs.kernel.org/networking/ip-sysctl.html).

`accept_redirects` controls receipt of ICMP routing advice. The Pi's configured default gateway and the router's forwarding policy remain in place. No `ip_forward`, NAT or firewall change is part of this fix. Flushing cached routes clears previously accepted redirects before opening a fresh connection.

## Verify routing, DNS and TCP

Check the effective policy and selected route:

```sh
sysctl net.ipv4.conf.all.accept_redirects \
       net.ipv4.conf.default.accept_redirects \
       net.ipv4.conf.wlan0.accept_redirects \
       net.ipv4.conf.eth0.accept_redirects
ip -brief address
ip route
ip route get 91.189.92.19
```

All four redirect values should be **0**. For an Internet destination, the route should use the configured Wi-Fi gateway: **10.0.0.1 on robotlab** or **10.3.1.1 on robomaster**, without an unexpected redirected next hop. The direct-cable subnet should add no Internet default route. The numerical destination above is the historical route-check example; HTTP verification below resolves its hostname afresh.

Then test DNS and bounded HTTP/HTTPS connections:

```sh
getent ahostsv4 ports.ubuntu.com
curl -4I --connect-timeout 10 --max-time 20 http://ports.ubuntu.com/ubuntu-ports/
curl -4I --connect-timeout 10 --max-time 20 https://ports.ubuntu.com/ubuntu-ports/
sudo apt-get update
```

The `-4` options isolate the IPv4 path for diagnosis. The historical fix did not install a persistent APT force-IPv4 option, proxy, replacement mirror or firewall workaround. An SSH SOCKS tunnel was used temporarily during diagnosis and removed afterward.

After the next planned reboot, repeat the sysctl, route and HTTPS checks to confirm persistence. The original bring-up recorded successful direct HTTP/HTTPS and APT access after applying the fix and again after reboot.

## If the symptom returns

Confirm the active NetworkManager profile, gateway and effective sysctl values first. Look for another configuration overriding the redirect policy. To observe incoming IPv4 redirects on the Pi's Wi-Fi interface:

```sh
nmcli device status
sudo tcpdump -ni wlan0 'icmp[0] == 5'
```

Stop the capture with Ctrl-C after reproducing the connection failure. Incoming redirects may still appear in the capture when the Pi is correctly ignoring them; use the effective settings and selected route to determine whether they were accepted. Keep fresh routing evidence before changing another component.
