#!/usr/bin/env bash
# Встановлення «Желейок» для поточного користувача (sudo не потрібен).
# Програма копіюється в ~/.local/share/jellies, з'являється команда `jellies`
# і пункт «Желейки» в меню програм.
set -euo pipefail

SRC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_HOME="${XDG_DATA_HOME:-$HOME/.local/share}"
DEST="$DATA_HOME/jellies"
BIN_DIR="$HOME/.local/bin"
APP_DIR="$DATA_HOME/applications"
DESKTOP="$APP_DIR/jellies.desktop"

if [ ! -f "$SRC_DIR/jellies" ]; then
    echo "Помилка: поруч зі скриптом немає файлу «jellies». Запустіть install.sh з розпакованої папки Jellies." >&2
    exit 1
fi

echo "Встановлюю «Желейки» в $DEST …"
mkdir -p "$DEST" "$BIN_DIR" "$APP_DIR"
for f in jellies icon.png README.txt uninstall.sh; do
    if [ -f "$SRC_DIR/$f" ]; then
        cp -f "$SRC_DIR/$f" "$DEST/$f"
    fi
done
chmod +x "$DEST/jellies"
if [ -f "$DEST/uninstall.sh" ]; then
    chmod +x "$DEST/uninstall.sh"
fi
ln -sf "$DEST/jellies" "$BIN_DIR/jellies"

cat > "$DESKTOP" <<DESKTOP_ENTRY
[Desktop Entry]
Type=Application
Version=1.0
Name=Желейки
Name[uk]=Желейки
Name[en]=Jellies
Comment=3D-симуляція м'яких желейок
Comment[en]=3D soft-body jelly sandbox
Exec="$DEST/jellies"
Path=$DEST
Icon=$DEST/icon.png
Terminal=false
Categories=Game;Simulation;
Keywords=jelly;soft body;physics;желе;желейки;
StartupNotify=true
DESKTOP_ENTRY
chmod +x "$DESKTOP"

if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$APP_DIR" >/dev/null 2>&1 || true
fi

echo
echo "Готово! «Желейки» встановлено."
echo "  • Запуск: знайдіть «Желейки» в меню програм"
echo "  • або в терміналі: jellies   (повний екран: jellies --fullscreen)"
case ":$PATH:" in
    *":$BIN_DIR:"*) ;;
    *)
        echo
        echo "Увага: папки $BIN_DIR немає в PATH, тому команда «jellies» у терміналі поки не знайдеться."
        echo "Додайте її так (і відкрийте новий термінал):"
        echo "  echo 'export PATH=\"\$HOME/.local/bin:\$PATH\"' >> ~/.bashrc"
        echo "Або запускайте напряму: $DEST/jellies"
        ;;
esac
echo
echo "Видалити програму: $DEST/uninstall.sh"
