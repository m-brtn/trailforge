# README menu and lake portrait: Expedition, detailed trees, morning light, Ultra.
# ./build/release/trailforge --script tools/scenarios/readme_shots.txt --no-audio
# Convert menu and lake_final captures to docs/menu.jpg and docs/lake.jpg:
# sips -s format jpeg -s formatOptions 90 --resampleWidth 1600 INPUT.png --out OUTPUT.jpg
# The separate expedition_photos.txt reproduces the approved forest hero.
controls pad_type none
set ui 0
set hud 0
set auto_reverse 0
set minimap 0
set compass 0
resize 1600 900
level exploration_peaks
until level == 1 20
menu pause
wait .2
menu settings
wait .2
menu select tree_style
wait .2
tap Right
wait .5
expect setting_tree_style == 1
until level == 1 30
menu back
wait .1
menu resume
wait 1
set ui 0
set hud 0
vehicle trailforge_expedition
quality Ultra
set flowers 1
set grass_dist 160
set motion_blur 1
set shafts .6
set flare .08
set fog .65
set valley_fog .9
set headlights 0
time 7.2
set sun_heading 208
set mud .35
set body_mud .55
menu main
until menu_fade == 1 8
time 7.8
set sun_heading 81
expect menu_screen == 1
expect menu_page == 0
wait 14.5
capture /tmp/trailforge_readme_01_menu.png
wait .3
menu start
wait .5
set ui 0
set hud 0
set headlights 0
time 8.2

set sun_heading 300
place -145 0 0
down S
wait 1.5
camera local -4 2.3 7 0 .5 0 42
wait .6
capture /tmp/trailforge_readme_lake_final.png
wait .3
expect body_glb == 1
up S
quit
