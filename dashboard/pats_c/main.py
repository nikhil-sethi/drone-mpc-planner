from flask.helpers import url_for
from flask_login import current_user
from flask import Blueprint, render_template, redirect, url_for, request
from . import db


main = Blueprint('main', __name__)

@main.route('/')
def index():
    if current_user.is_authenticated:
        return redirect(url_for('/pats-c/'))
    else:
        return redirect(url_for('auth.login'))

@main.before_request
def login_in_check():
    if (not current_user.is_authenticated or request.endpoint == 'login'):
        return redirect(url_for('auth.login'))