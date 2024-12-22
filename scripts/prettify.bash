set -e -x

TARGET=$1

if [ -z "$1" ]
  then
    echo "Formatting all of zaapp/"
    TARGET="zaapp/"
else
  TARGET="$1"
fi

echo ""
echo "ruff:"
ruff check --extend-select I --fix "$TARGET"
ruff format "$TARGET"

