#!/usr/bin/env bash
# Видалення «Желейок», встановлених скриптом install.sh.
set -euo pipefail

DATA_HOME="${XDG_DATA_HOME:-$HOME/.local/share}"
DEST="$DATA_HOME/jellies"
BIN_LINK="$HOME/.local/bin/jellies"
APP_DIR="$DATA_HOME/applications"
DESKTOP="$APP_DIR/jellies.desktop"

# Remove the command only if it is our symlink
if [ -L "$BIN_LINK" ] && [ "$(readlink "$BIN_LINK")" = "$DEST/jellies" ]; then
    rm -f "$BIN_LINK"
fi
rm -f "$DESKTOP"
if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$APP_DIR" >/dev/null 2>&1 || true
fi

# Remove only the install folder created by install.sh
if [ -d "$DEST" ] && [ -f "$DEST/jellies" ] && [ "$(basename "$DEST")" = "jellies" ]; then
    rm -rf -- "$DEST"
fi

echo "«Желейки» видалено. Дякуємо, що грали!"
