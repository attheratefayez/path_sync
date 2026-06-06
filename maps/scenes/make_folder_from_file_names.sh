for f in *.*; do
    d="${f%.*}-${f##*.}"
    mkdir -p "$d"
    mv "$f" "$d/"
done
