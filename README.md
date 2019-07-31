# PATS


Convert first 10s videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -ss 00:00:00 -to 00:00:10 -c:v copy {}.mp4 \;` 

Convert videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} {}.mp4 \; -exec rm {} \;` 

Re-encode old videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v libx264 -level 3.0 -pix_fmt yuv420p -crf 21 -preset slow {}_reencoded.mp4 \; -exec rm {} \;` 

Re-encode color result video's (not necessary anymore):

`find -iname videoResult.avi -exec ffmpeg -i {} -c:v libx264 -preset slow -pix_fmt yuv420p -profile:v high -level 4.0 -b:v 15M -bf 2 -crf 18 {}.mp4 \;`

`find -iname videoResult.avi -exec ffmpeg -i {} -f matroska -c:v libvpx -crf 21 -b:v 15M {}.mkv \;`

