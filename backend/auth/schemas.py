from pydantic import BaseModel, EmailStr
from typing import Optional, Dict, Any
from datetime import datetime

class UserBase(BaseModel):
    email: EmailStr

class UserCreate(UserBase):
    password: str
    first_name: str
    last_name: str
    experience_level: Optional[str] = None
    software_skills: Optional[Dict[str, Any]] = None
    hardware_access: Optional[Dict[str, Any]] = None
    learning_preferences: Optional[Dict[str, Any]] = None

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class UserResponse(UserBase):
    id: str
    is_active: bool
    email_verified: bool
    is_onboarded: bool
    created_at: datetime

    class Config:
        from_attributes = True

class Token(BaseModel):
    access_token: str
    token_type: str

class TokenData(BaseModel):
    user_id: Optional[str] = None

class UserProfileUpdate(BaseModel):
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    experience_level: Optional[str] = None
    software_skills: Optional[Dict[str, Any]] = None
    hardware_access: Optional[Dict[str, Any]] = None
    learning_preferences: Optional[Dict[str, Any]] = None

class PersonalizationSettings(BaseModel):
    difficulty_level: Optional[str] = None
    content_depth: Optional[str] = None
    example_preference: Optional[str] = None