from sqlalchemy import Column, Integer, String, Boolean, DateTime, ForeignKey, JSON
from sqlalchemy.ext.mutable import MutableDict
from .database import Base
from datetime import datetime
import uuid


class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    is_active = Column(Boolean, default=True)
    email_verified = Column(Boolean, default=False)
    is_onboarded = Column(Boolean, default=False)


class UserProfile(Base):
    __tablename__ = "user_profiles"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))  # Primary key for profile
    user_id = Column(String, ForeignKey("users.id"), nullable=False, unique=True)  # Foreign key to user
    first_name = Column(String)
    last_name = Column(String)
    experience_level = Column(String)  # 'beginner', 'intermediate', 'advanced'
    software_skills = Column(MutableDict.as_mutable(JSON))
    hardware_access = Column(MutableDict.as_mutable(JSON))
    learning_preferences = Column(MutableDict.as_mutable(JSON))
    personalization_settings = Column(MutableDict.as_mutable(JSON))
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)