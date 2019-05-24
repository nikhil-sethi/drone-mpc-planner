# PATS


Re-encode:  
`find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v libx264 -level 3.0 -pix_fmt yuv420p -crf 21 -preset slow {}_reencoded.mp4 \;`  
`find -iname videoRawLR.avi -exec rm  {} \;`  

