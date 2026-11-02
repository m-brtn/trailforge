# README screenshots: the start menu over Exploration Peaks, then two chase views from behind the SUV on the level
# (Valley Road ford over Glacier Creek at golden hour, the Alpine Lake meadow on a soft morning), Ultra, no HUD:
#   ./build/release/trailforge --script tools/scenarios/readme_shots.txt --no-audio
# Captures /tmp/trailforge_readme_*.png at a 1600x900 point window (3200x1800 px on a Retina display). The JPEGs in
# docs/ are these captures scaled to 1600 px wide:
#   sips -s format jpeg -s formatOptions 82 --resampleWidth 1600 /tmp/trailforge_readme_01_menu.png --out docs/menu.jpg
#   (02_ford -> docs/ford.jpg, 03_meadow -> docs/meadow.jpg)
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

# ---------------------------------------------------------------- Valley Road ford, golden hour
time 17.9
place -222 -334 100
wait 2
camera local 2.2 2.3 -7.5 0 0.9 8 55
wait 2
expect wheels_contact == 4
capture /tmp/trailforge_readme_02_ford.png
wait 0.3

# ---------------------------------------------------------------- Alpine Lake meadow, morning
time 7.6
place -170 62 160
wait 2
camera local 1.8 2.2 -7.5 0 0.9 8 55
wait 2
expect wheels_contact == 4
capture /tmp/trailforge_readme_03_meadow.png
wait 0.3
camera follow
quit
