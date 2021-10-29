import os
import sys
import logging
import logging.handlers
from flask import Flask, send_from_directory
from flask_sqlalchemy import SQLAlchemy
from flask_login import login_required, LoginManager
from pathlib import Path
sys.path.append('pats_c/lib')  # noqa
import lib_patsc as patsc

from . import pats_c, system_widget

db = SQLAlchemy()
dash_c = pats_c.dash_application()
dash_sys_wid = system_widget.dash_application()

Path(Path(patsc.patsc_error_log).parent.absolute()).mkdir(parents=True, exist_ok=True)
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
error_file_handler = logging.handlers.RotatingFileHandler(filename=patsc.patsc_error_log, maxBytes=1024 * 1024 * 100, backupCount=1)
error_file_handler.setFormatter(file_format)
error_file_handler.level = logging.ERROR


def create_app():
    current_folder = os.path.dirname(os.path.realpath(__file__))
    server = Flask(__name__, static_folder=os.path.join(current_folder, 'assets'))
    server.logger.addHandler(error_file_handler)
    db_path = os.path.expanduser('~/patsc/db/pats_creds.db')
    config_path = os.path.expanduser('~/patsc/db/.pats-c-key.py')
    db_uri = 'sqlite:///{}'.format(db_path)
    server.config.from_pyfile(config_path)
    server.config['SQLALCHEMY_DATABASE_URI'] = db_uri
    server.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

    db.init_app(server)

    login_manager = LoginManager()
    login_manager.login_view = 'auth.login'
    login_manager.init_app(server)

    dash_c.init_app(app=server)
    dash_sys_wid.init_app(app=server)

    for view_func in server.view_functions:
        if view_func.startswith(dash_c.config['url_base_pathname']):
            server.view_functions[view_func] = login_required(server.view_functions[view_func])
        if view_func.startswith(dash_sys_wid.config['url_base_pathname']):
            server.view_functions[view_func] = login_required(server.view_functions[view_func])

    from .models import User

    @login_manager.user_loader
    def load_user(user_id):  # pylint: disable=unused-variable
        return User.query.get(int(user_id))

    from .auth import auth as auth_blueprint
    server.register_blueprint(auth_blueprint)

    from .main import main as main_blueprint
    server.register_blueprint(main_blueprint)

    # Function is necessary for the video requests of pats-c
    @server.route('/static/<path:filename>')
    @login_required
    def base_static(filename):  # pylint: disable=unused-variable
        return send_from_directory(os.path.join('../static/'), filename)

    return server
