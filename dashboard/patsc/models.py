# mypy: ignore-errors
from flask_login import UserMixin
from . import db


class User(UserMixin, db.Model):
    id = db.Column(db.Integer, primary_key=True)  # pylint: disable=no-member
    username = db.Column(db.String(100), unique=True)  # pylint: disable=no-member
    password = db.Column(db.String(100))  # pylint: disable=no-member
