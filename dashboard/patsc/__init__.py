import os
import sys
import logging
import logging.handlers
from flask import Flask, send_from_directory
from flask_sqlalchemy import SQLAlchemy
from flask_login import login_required, LoginManager
from pathlib import Path
sys.path.append('patsc/lib')  # noqa
import lib_patsc as pc

from . import patsc, system_widget, live, timelapse

if not os.path.exists(os.path.expanduser('~/patsc/static/')):
    os.makedirs(os.path.expanduser('~/patsc/static/'))

db = SQLAlchemy()
patsc_view = patsc.dash_application()
sys_wid_view = system_widget.dash_application()
live_view = live.dash_application()
timelapse_view = timelapse.dash_application()

Path(Path(pc.patsc_error_log).parent.absolute()).mkdir(parents=True, exist_ok=True)
file_format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
error_file_handler = logging.handlers.RotatingFileHandler(filename=pc.patsc_error_log, maxBytes=1024 * 1024 * 100, backupCount=1)
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

    patsc_view.init_app(app=server)
    sys_wid_view.init_app(app=server)
    live_view.init_app(app=server)
    timelapse_view.init_app(app=server)

    for view_func in server.view_functions:
        if view_func.startswith(patsc_view.config['url_base_pathname']):
            server.view_functions[view_func] = login_required(server.view_functions[view_func])
        if view_func.startswith(sys_wid_view.config['url_base_pathname']):
            server.view_functions[view_func] = login_required(server.view_functions[view_func])
        if view_func.startswith(live_view.config['url_base_pathname']):
            server.view_functions[view_func] = login_required(server.view_functions[view_func])
        if view_func.startswith(timelapse_view.config['url_base_pathname']):
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
