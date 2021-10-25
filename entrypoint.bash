set -e
tail -f /launcher/logs &
tail -f /gzweb/logs &
exec /usr/bin/supervisord -c /etc/supervisor/supervisord.conf
