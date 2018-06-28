# PATS



http://mikelococo.com/2008/01/multihop-ssh/ 
ssh -i ~/.ssh/id_rsa kevin@131.180.117.41 -t "ssh pats@localhost -p 16668 -i~/.ssh/id_rsa"
https://stackoverflow.com/questions/25084288/keep-ssh-session-alive

find . -type d -newermt "2018-06-26 17:00:00" ! -newermt "2018-06-27 09:00:00" -exec du -hd1 {} \;

find . -type d -newermt "2018-06-26 17:00:00" ! -newermt "2018-06-27 09:00:00" -exec tar -rvf test1.tar {} \; && tar -czvf test1.tar.gz test1.tar

cd ~
rm data_copy -r
cp -Rp data data_copy
cd data_copy
find -iname *.txt -exec rm {} \;
find -size  0 -exec rm {} \;
find . -empty -type d -delete

find . -type f -newermt "2018-06-22 18:00:00" ! -newermt "2018-06-23 07:00:00" -exec tar -rvf day1.tar {} \;
find . -type f -newermt "2018-06-23 18:00:00" ! -newermt "2018-06-24 07:00:00" -exec tar -rvf day2.tar {} \;
find . -type f -newermt "2018-06-24 18:00:00" ! -newermt "2018-06-25 07:00:00" -exec tar -rvf day3.tar {} \;
find . -type f -newermt "2018-06-25 18:00:00" ! -newermt "2018-06-26 07:00:00" -exec tar -rvf day4.tar {} \;
find . -type f -newermt "2018-06-26 18:00:00" ! -newermt "2018-06-27 07:00:00" -exec tar -rvf day5.tar {} \;
find . -type f -newermt "2018-06-27 18:00:00" ! -newermt "2018-06-28 07:00:00" -exec tar -rvf day6.tar {} \;

find -iname videoRawLR.avi -exec ffmpeg -i {} -c:v copy -crf 21 -preset slow {}_2.avi \;
