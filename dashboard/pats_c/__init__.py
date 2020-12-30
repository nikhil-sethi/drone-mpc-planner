import os
from flask import Flask, send_from_directory
from flask_sqlalchemy import SQLAlchemy
from flask_login import login_required,LoginManager
from . import pats_c

db = SQLAlchemy()
dash = pats_c.dash_application()

def create_app():

    current_folder = os.path.dirname(os.path.realpath(__file__))
    parent_folder = os.path.realpath(os.path.join(current_folder,os.pardir))
    app = Flask(__name__, static_folder=os.path.join(current_folder,'assets'))

    db_path = os.path.join(os.path.expanduser('~'), 'pats_creds.db')
    config_path = os.path.join(os.path.expanduser('~'), '.pats-c-key.py')
    db_uri = 'sqlite:///{}'.format(db_path)
    app.config.from_pyfile(config_path)
    app.config['SQLALCHEMY_DATABASE_URI'] = db_uri
    app.config['SQLALCHEMY_TRACK_MODIFICATIONS'] = False

    db.init_app(app)

    login_manager = LoginManager()
    login_manager.login_view = 'auth.login'
    login_manager.init_app(app)

    dash.init_app(app=app)

    for view_func in app.view_functions:
       if view_func.startswith(dash.config['url_base_pathname']):
            app.view_functions[view_func] = login_required(app.view_functions[view_func])

    from .models import User

    @login_manager.user_loader
    def load_user(user_id):
        return User.query.get(int(user_id))

    from .auth import auth as auth_blueprint
    app.register_blueprint(auth_blueprint)

    from .main import main as main_blueprint
    app.register_blueprint(main_blueprint)

    # Function is necessary for the video requests of pats-c
    @app.route('/static/<path:filename>')
    @login_required
    def base_static(filename):
        return send_from_directory(os.path.join(parent_folder,'static'), filename)

    return app
