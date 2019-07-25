# PATS


Re-encode videoLR.avi:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v libx264 -level 3.0 -pix_fmt yuv420p -crf 21 -preset slow {}_reencoded.mp4 \; -exec rm {} \;`  

Re-encode color result video's:

`find -iname videoResult.avi -exec ffmpeg -i {} -c:v libx264 -preset slow -pix_fmt yuv420p -profile:v high -level 4.0 -b:v 15M -bf 2 -crf 18 videoResult.mp4 \;`
