# README start-menu screenshot over Exploration Peaks. Action views are captured separately by
# expedition_photos.txt (forest.jpg) and expedition_meadow_photos.txt (meadow.jpg):
#   ./build/release/trailforge --script tools/scenarios/readme_shots.txt --no-audio
# Captures /tmp/trailforge_readme_*.png at a 1600x900 point window (3200x1800 px on a Retina display). The JPEGs in
# docs/ are these captures scaled to 1600 px wide:
#   sips -s format jpeg -s formatOptions 82 --resampleWidth 1600 /tmp/trailforge_readme_01_menu.png --out docs/menu.jpg

set ui 0
set hud 0
set auto_reverse 0
resize 1600 900
level exploration_peaks
until level == 1 10
quality Ultra
set headlights 0

# ---------------------------------------------------------------- start menu (cinematic camera over the attract drive)
menu main
until menu_fade == 1 8
expect menu_screen == 1
expect menu_page == 0
wait 14.5                       # the attract drive in the Glacier Creek ford
capture /tmp/trailforge_readme_01_menu.png
wait 0.3
menu start
wait 0.5
expect menu_screen == 2
set ui 0
set hud 0
set headlights 0

quit
