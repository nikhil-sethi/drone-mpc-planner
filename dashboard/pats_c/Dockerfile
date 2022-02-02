FROM python:3.8.10-buster
RUN apt-get update && \
  apt-get install -y openssh-client rsync ffmpeg
RUN pip3 install --upgrade pip
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

RUN groupadd flaskgroup && useradd -m -g flaskgroup -s /bin/bash flask
RUN chown -R flask:flaskgroup /home/flask
USER flask

RUN mkdir -p /home/flask/pats_c
RUN mkdir -p /home/flask/patsc/db/
RUN mkdir -p /home/flask/patsc/static/
RUN mkdir -p /home/flask/patsc/logs/
RUN mkdir -p /home/flask/.ssh
RUN ln -s /home/flask/patsc/base_install/sshconfig /home/flask/.ssh/config
RUN ln -s /home/flask/patsc/ssh_tmp/id_rsa /home/flask/.ssh/id_rsa
RUN ln -s /home/flask/patsc/ssh_tmp/id_rsa.pub /home/flask/.ssh/id_rsa.pub
RUN ln -s /home/flask/patsc/ssh_tmp/id_rsa /home/flask/.ssh/pats_id_rsa
RUN ln -s /home/flask/patsc/ssh_tmp/authorized_keys /home/flask/.ssh/authorized_keys
RUN ln -s /home/flask/patsc/ssh_tmp/known_hosts /home/flask/.ssh/known_hosts
RUN ln -s /home/flask/patsc/ssh_tmp/id_rsa.pub /home/flask/.ssh/pats_id_rsa.pub
RUN ln -s /home/flask/pats_c/static/ /home/flask/static
WORKDIR /home/flask/

ENV PYTHONUNBUFFERED=TRUE

COPY . /home/flask/pats_c