mkdir /etc/cv_run/
sudo ln -s ./build/cv_run /etc/cv_run/cv_run
sudo ln -s ./start.sh /etc/cv_run/start.sh
sudo chmod 777 /etc/cv_run/start.sh
sudo chmod 777 /etc/cv_run/cv_run
sudo ln -s ./cvauto.service /etc/systemd/system/cvauto.service
sudo chmod 777 /etc/systemd/system/cvauto.service
sudo systemctl daemon-reload

# sudo systemctl enable cvauto.service