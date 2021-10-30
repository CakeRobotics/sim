set -e
touch /sim/launcher/logs /sim/gzweb/logs
tail -f /sim/launcher/logs &
tail -f /sim/gzweb/logs &
exec /usr/bin/supervisord -c /sim/supervisord.conf
