from flask import Blueprint, render_template, redirect, url_for, request, flash
from werkzeug.security import generate_password_hash, check_password_hash
from flask_login import login_user, logout_user, login_required
from .models import User
from . import db
import urllib


auth = Blueprint('auth', __name__)


@auth.route('/login')
def login():
    return render_template('login.html')


@auth.route('/login', methods=['POST'])
def login_post():
    username = request.form.get('username')
    password = request.form.get('password')
    remember = True if request.form.get('remember') else False

    user = User.query.filter_by(username=username).first()

    splitted_url = request.referrer.split('next=')
    if len(splitted_url) > 1:
        requested_page = urllib.parse.unquote(splitted_url[1])
    else:
        requested_page = None

    if not user or not check_password_hash(user.password, password):
        flash('Please check your login details and try again.')
        return redirect(url_for('auth.login'))

    login_user(user, remember=remember)
    if requested_page:
        return redirect(url_for(requested_page))
    else:
        return redirect(url_for('/pats-c/'))


@auth.route('/logout')
@login_required
def logout():
    logout_user()
    return redirect(url_for('main.index'))

# These methods can be use to create new users. We only want to do that internally right now.


@auth.route('/bram')
@login_required
def signup():
    return render_template('signup.html')


@auth.route('/bram', methods=['POST'])
def signup_post():
    username = request.form.get('username')
    password = request.form.get('password')

    user = User.query.filter_by(username=username).first()
    if user:
        flash('Username address already exists')
        return redirect(url_for('auth.signup'))

    new_user = User(username=username, password=generate_password_hash(password, method='sha256'))

    db.session.add(new_user)  # pylint: disable=no-member
    db.session.commit()  # pylint: disable=no-member
    flash('User added!')

    return redirect(url_for('auth.signup'))
