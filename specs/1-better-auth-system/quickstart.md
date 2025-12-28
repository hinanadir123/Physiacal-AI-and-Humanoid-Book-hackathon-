# Quickstart: Better-Auth Authentication System for Textbook

## Overview
This guide provides step-by-step instructions for implementing the Better-Auth authentication system into the existing Docusaurus textbook project. The system includes user registration with profile questions, login functionality, and personalization features.

## Prerequisites
- Node.js 18+ (for Docusaurus frontend)
- Python 3.11+ (for backend API)
- PostgreSQL 12+ (or Docker for easy setup)
- Git for version control

## Step 1: Project Setup

### 1.1 Clone and Prepare the Repository
```bash
# If not already done, ensure you're in the project directory
cd C:\repo\raat-hackathon

# Create backend directory
mkdir backend
cd backend
```

### 1.2 Backend Setup
```bash
# Initialize Python project
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Create requirements.txt
cat > requirements.txt << 'EOF'
fastapi==0.104.1
uvicorn[standard]==0.24.0
sqlalchemy==2.0.23
psycopg2-binary==2.9.9
pydantic==2.5.0
pydantic-settings==2.1.0
better-exceptions==0.3.2
alembic==1.13.1
python-jose[cryptography]==3.3.0
passlib[bcrypt]==1.7.4
python-multipart==0.0.6
cryptography==41.0.8
email-validator==2.1.0
EOF

# Install dependencies
pip install -r requirements.txt
```

## Step 2: Database Schema

### 2.1 Create Database Models
Create the following files in the backend directory:

```python
# backend/models/__init__.py
from .database import Base, engine
from .user import User, UserProfile

def init_db():
    """Initialize the database tables"""
    Base.metadata.create_all(bind=engine)
```

```python
# backend/models/database.py
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
import os
from pydantic_settings import BaseSettings

class Settings(BaseSettings):
    database_url: str = "postgresql://user:password@localhost/auth_system"

    class Config:
        env_file = ".env"

settings = Settings()
engine = create_engine(settings.database_url)
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
```

```python
# backend/models/user.py
from sqlalchemy import Column, Integer, String, Boolean, DateTime, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.mutable import MutableDict
from sqlalchemy.dialects.postgresql import JSONB
from .database import Base
from datetime import datetime
import uuid

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String, unique=True, index=True, nullable=False)
    hashed_password = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    is_active = Column(Boolean, default=True)
    email_verified = Column(Boolean, default=False)
    is_onboarded = Column(Boolean, default=False)

class UserProfile(Base):
    __tablename__ = "user_profiles"

    user_id = Column(UUID(as_uuid=True), primary_key=True, ForeignKey("users.id"), nullable=False)
    first_name = Column(String)
    last_name = Column(String)
    experience_level = Column(String)  # 'beginner', 'intermediate', 'advanced'
    software_skills = Column(MutableDict.as_mutable(JSONB))
    hardware_access = Column(MutableDict.as_mutable(JSONB))
    learning_preferences = Column(MutableDict.as_mutable(JSONB))
    personalization_settings = Column(MutableDict.as_mutable(JSONB))
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
```

## Step 3: Backend API Implementation

### 3.1 Create Authentication Utilities
```python
# backend/auth/utils.py
from passlib.context import CryptContext
from datetime import datetime, timedelta
from typing import Optional
from jose import JWTError, jwt
import os

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")

SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            return None
        return user_id
    except JWTError:
        return None
```

### 3.2 Create Pydantic Schemas
```python
# backend/auth/schemas.py
from pydantic import BaseModel, EmailStr
from typing import Optional, List, Dict, Any
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
```

### 3.3 Create Authentication Routes
```python
# backend/auth/routes.py
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from sqlalchemy.orm import Session
from typing import Optional
from ..models.user import User, UserProfile
from ..models.database import get_db
from .utils import verify_password, get_password_hash, create_access_token, verify_token
from .schemas import UserCreate, UserLogin, UserResponse, Token, UserProfileUpdate, PersonalizationSettings
import uuid

router = APIRouter(prefix="/api/v1/auth", tags=["auth"])
oauth2_scheme = OAuth2PasswordBearer(tokenUrl="token")

def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)):
    user_id = verify_token(token)
    if user_id is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    user = db.query(User).filter(User.id == user_id).first()
    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found",
            headers={"WWW-Authenticate": "Bearer"},
        )
    return user

@router.post("/register", response_model=UserResponse)
def register(user: UserCreate, db: Session = Depends(get_db)):
    # Check if user already exists
    existing_user = db.query(User).filter(User.email == user.email).first()
    if existing_user:
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail="Email already registered"
        )

    # Create new user
    hashed_password = get_password_hash(user.password)
    db_user = User(
        email=user.email,
        hashed_password=hashed_password
    )
    db.add(db_user)
    db.commit()
    db.refresh(db_user)

    # Create user profile
    profile = UserProfile(
        user_id=db_user.id,
        first_name=user.first_name,
        last_name=user.last_name,
        experience_level=user.experience_level,
        software_skills=user.software_skills or {},
        hardware_access=user.hardware_access or {},
        learning_preferences=user.learning_preferences or {}
    )
    db.add(profile)
    db.commit()

    return db_user

@router.post("/login", response_model=Token)
def login(user_credentials: UserLogin, db: Session = Depends(get_db)):
    user = db.query(User).filter(User.email == user_credentials.email).first()
    if not user or not verify_password(user_credentials.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect email or password",
            headers={"WWW-Authenticate": "Bearer"},
        )

    access_token = create_access_token(data={"sub": str(user.id)})
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/logout")
def logout():
    # In a real implementation, you might add the token to a blacklist
    return {"message": "Successfully logged out"}
```

### 3.4 Create Profile Routes
```python
# backend/api/v1/profiles.py
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from ..models.user import User, UserProfile
from ..models.database import get_db
from ..auth.routes import get_current_user
from ..auth.schemas import UserProfileUpdate, PersonalizationSettings

router = APIRouter(prefix="/api/v1/profiles", tags=["profiles"])

@router.get("/me")
def get_current_profile(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(UserProfile).filter(UserProfile.user_id == current_user.id).first()
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )
    return profile

@router.put("/me")
def update_profile(
    profile_update: UserProfileUpdate,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(UserProfile).filter(UserProfile.user_id == current_user.id).first()
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    # Update profile fields
    for field, value in profile_update.dict(exclude_unset=True).items():
        setattr(profile, field, value)

    db.commit()
    db.refresh(profile)
    return profile

@router.put("/me/personalization")
def update_personalization_settings(
    settings: PersonalizationSettings,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(UserProfile).filter(UserProfile.user_id == current_user.id).first()
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    if not profile.personalization_settings:
        profile.personalization_settings = {}

    # Update personalization settings
    for field, value in settings.dict(exclude_unset=True).items():
        profile.personalization_settings[field] = value

    db.commit()
    db.refresh(profile)
    return profile

@router.get("/me/personalization")
def get_personalization_settings(
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    profile = db.query(UserProfile).filter(UserProfile.user_id == current_user.id).first()
    if not profile:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Profile not found"
        )

    return profile.personalization_settings or {}
```

### 3.5 Create Main Application
```python
# backend/app.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .auth.routes import router as auth_router
from .api.v1.profiles import router as profile_router
from .models import init_db

app = FastAPI(title="Better-Auth API", version="1.0.0")

# Add CORS middleware (adjust origins for production)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Adjust for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(auth_router)
app.include_router(profile_router)

@app.on_event("startup")
def on_startup():
    init_db()

@app.get("/")
def read_root():
    return {"message": "Better-Auth API is running!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)
```

## Step 4: Frontend Integration

### 4.1 Install Frontend Dependencies
```bash
# Navigate to the Docusaurus root directory (likely the project root)
cd C:\repo\raat-hackathon

# Install authentication-related dependencies
npm install axios react-query @docusaurus/core
```

### 4.2 Create Frontend Utilities
```javascript
// website/src/utils/auth.js
import axios from 'axios';

const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

const api = axios.create({
  baseURL: API_BASE_URL,
  headers: {
    'Content-Type': 'application/json',
  },
});

// Add token to requests if available
api.interceptors.request.use((config) => {
  const token = localStorage.getItem('auth_token');
  if (token) {
    config.headers.Authorization = `Bearer ${token}`;
  }
  return config;
});

// Add response interceptor to handle token expiration
api.interceptors.response.use(
  (response) => response,
  (error) => {
    if (error.response?.status === 401) {
      localStorage.removeItem('auth_token');
      window.location.href = '/auth/signin';
    }
    return Promise.reject(error);
  }
);

export const authAPI = {
  // Authentication
  register: (userData) => api.post('/api/v1/auth/register', userData),
  login: (credentials) => api.post('/api/v1/auth/login', credentials),
  logout: () => api.post('/api/v1/auth/logout'),

  // Profile management
  getCurrentProfile: () => api.get('/api/v1/profiles/me'),
  updateProfile: (profileData) => api.put('/api/v1/profiles/me', profileData),

  // Personalization
  getPersonalizationSettings: () => api.get('/api/v1/profiles/me/personalization'),
  updatePersonalizationSettings: (settings) =>
    api.put('/api/v1/profiles/me/personalization', settings),
};

export const setAuthToken = (token) => {
  if (token) {
    localStorage.setItem('auth_token', token);
  } else {
    localStorage.removeItem('auth_token');
  }
};

export const getAuthToken = () => {
  return localStorage.getItem('auth_token');
};

export const isAuthenticated = () => {
  return !!getAuthToken();
};
```

### 4.3 Create Signup Form Component
```jsx
// website/src/components/Auth/SignupForm.jsx
import React, { useState } from 'react';
import { authAPI, setAuthToken } from '../../utils/auth';
import './Auth.css';

const SignupForm = ({ onSignupSuccess }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: '',
    confirmPassword: '',
    firstName: '',
    lastName: '',
    experienceLevel: 'beginner',
    softwareSkills: {
      programming_languages: [],
      frameworks: [],
      tools: []
    },
    hardwareAccess: {
      development_kits: [],
      robotics_platforms: [],
      simulation_environments: []
    },
    learningPreferences: {
      content_types: [],
      learning_pace: 'moderate',
      preferred_topics: []
    }
  });

  const [errors, setErrors] = useState({});
  const [loading, setLoading] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSkillChange = (category, index, field, value) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: prev.softwareSkills[category].map((skill, i) =>
          i === index ? { ...skill, [field]: value } : skill
        )
      }
    }));
  };

  const addSkill = (category) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: [...prev.softwareSkills[category], { name: '', proficiency: 1 }]
      }
    }));
  };

  const removeSkill = (category, index) => {
    setFormData(prev => ({
      ...prev,
      softwareSkills: {
        ...prev.softwareSkills,
        [category]: prev.softwareSkills[category].filter((_, i) => i !== index)
      }
    }));
  };

  const handleHardwareChange = (category, value) => {
    setFormData(prev => ({
      ...prev,
      hardwareAccess: {
        ...prev.hardwareAccess,
        [category]: value.split(',').map(item => item.trim()).filter(Boolean)
      }
    }));
  };

  const validateForm = () => {
    const newErrors = {};

    if (!formData.email) newErrors.email = 'Email is required';
    if (!formData.password) newErrors.password = 'Password is required';
    if (formData.password.length < 8) newErrors.password = 'Password must be at least 8 characters';
    if (formData.password !== formData.confirmPassword) newErrors.confirmPassword = 'Passwords do not match';
    if (!formData.firstName) newErrors.firstName = 'First name is required';
    if (!formData.lastName) newErrors.lastName = 'Last name is required';

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) return;

    setLoading(true);

    try {
      const response = await authAPI.register({
        email: formData.email,
        password: formData.password,
        first_name: formData.firstName,
        last_name: formData.lastName,
        experience_level: formData.experienceLevel,
        software_skills: formData.softwareSkills,
        hardware_access: formData.hardwareAccess,
        learning_preferences: formData.learningPreferences
      });

      setAuthToken(response.data.access_token);
      onSignupSuccess && onSignupSuccess();
    } catch (error) {
      console.error('Registration error:', error);
      setErrors({ general: 'Registration failed. Please try again.' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signup-form-container">
      <h2>Create Your Account</h2>
      {errors.general && <div className="error-message">{errors.general}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={errors.email ? 'error' : ''}
          />
          {errors.email && <div className="error-text">{errors.email}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={errors.password ? 'error' : ''}
          />
          {errors.password && <div className="error-text">{errors.password}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="confirmPassword">Confirm Password</label>
          <input
            type="password"
            id="confirmPassword"
            name="confirmPassword"
            value={formData.confirmPassword}
            onChange={handleChange}
            className={errors.confirmPassword ? 'error' : ''}
          />
          {errors.confirmPassword && <div className="error-text">{errors.confirmPassword}</div>}
        </div>

        <div className="form-row">
          <div className="form-group">
            <label htmlFor="firstName">First Name</label>
            <input
              type="text"
              id="firstName"
              name="firstName"
              value={formData.firstName}
              onChange={handleChange}
              className={errors.firstName ? 'error' : ''}
            />
            {errors.firstName && <div className="error-text">{errors.firstName}</div>}
          </div>

          <div className="form-group">
            <label htmlFor="lastName">Last Name</label>
            <input
              type="text"
              id="lastName"
              name="lastName"
              value={formData.lastName}
              onChange={handleChange}
              className={errors.lastName ? 'error' : ''}
            />
            {errors.lastName && <div className="error-text">{errors.lastName}</div>}
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="experienceLevel">Experience Level</label>
          <select
            id="experienceLevel"
            name="experienceLevel"
            value={formData.experienceLevel}
            onChange={handleChange}
          >
            <option value="beginner">Beginner</option>
            <option value="intermediate">Intermediate</option>
            <option value="advanced">Advanced</option>
          </select>
        </div>

        <div className="form-section">
          <h3>Software Skills</h3>

          <div className="skills-category">
            <h4>Programming Languages</h4>
            {formData.softwareSkills.programming_languages.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Language name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('programming_languages', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('programming_languages', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('programming_languages', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('programming_languages')}>Add Language</button>
          </div>

          <div className="skills-category">
            <h4>Frameworks</h4>
            {formData.softwareSkills.frameworks.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Framework name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('frameworks', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('frameworks', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('frameworks', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('frameworks')}>Add Framework</button>
          </div>

          <div className="skills-category">
            <h4>Tools</h4>
            {formData.softwareSkills.tools.map((skill, index) => (
              <div key={index} className="skill-item">
                <input
                  type="text"
                  placeholder="Tool name"
                  value={skill.name}
                  onChange={(e) => handleSkillChange('tools', index, 'name', e.target.value)}
                />
                <select
                  value={skill.proficiency}
                  onChange={(e) => handleSkillChange('tools', index, 'proficiency', parseInt(e.target.value))}
                >
                  {[1, 2, 3, 4, 5].map(level => (
                    <option key={level} value={level}>{level}</option>
                  ))}
                </select>
                <button type="button" onClick={() => removeSkill('tools', index)}>Remove</button>
              </div>
            ))}
            <button type="button" onClick={() => addSkill('tools')}>Add Tool</button>
          </div>
        </div>

        <div className="form-section">
          <h3>Hardware Access</h3>

          <div className="form-group">
            <label>Development Kits (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., Raspberry Pi, Arduino, Jetson Nano"
              value={formData.hardwareAccess.development_kits.join(', ')}
              onChange={(e) => handleHardwareChange('development_kits', e.target.value)}
            />
          </div>

          <div className="form-group">
            <label>Robotics Platforms (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., ROS 2, NVIDIA Isaac, ROS 1"
              value={formData.hardwareAccess.robotics_platforms.join(', ')}
              onChange={(e) => handleHardwareChange('robotics_platforms', e.target.value)}
            />
          </div>

          <div className="form-group">
            <label>Simulation Environments (comma separated)</label>
            <input
              type="text"
              placeholder="e.g., Gazebo, Unity, Webots"
              value={formData.hardwareAccess.simulation_environments.join(', ')}
              onChange={(e) => handleHardwareChange('simulation_environments', e.target.value)}
            />
          </div>
        </div>

        <div className="form-group">
          <label htmlFor="learningPace">Learning Pace</label>
          <select
            id="learningPace"
            name="learningPace"
            value={formData.learningPreferences.learning_pace}
            onChange={(e) => setFormData(prev => ({
              ...prev,
              learningPreferences: {
                ...prev.learningPreferences,
                learning_pace: e.target.value
              }
            }))}
          >
            <option value="slow">Slow</option>
            <option value="moderate">Moderate</option>
            <option value="fast">Fast</option>
          </select>
        </div>

        <button type="submit" disabled={loading} className="submit-button">
          {loading ? 'Creating Account...' : 'Create Account'}
        </button>
      </form>
    </div>
  );
};

export default SignupForm;
```

### 4.4 Create Signin Form Component
```jsx
// website/src/components/Auth/SigninForm.jsx
import React, { useState } from 'react';
import { authAPI, setAuthToken } from '../../utils/auth';
import './Auth.css';

const SigninForm = ({ onSigninSuccess }) => {
  const [formData, setFormData] = useState({
    email: '',
    password: ''
  });

  const [errors, setErrors] = useState({});
  const [loading, setLoading] = useState(false);

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const validateForm = () => {
    const newErrors = {};

    if (!formData.email) newErrors.email = 'Email is required';
    if (!formData.password) newErrors.password = 'Password is required';

    setErrors(newErrors);
    return Object.keys(newErrors).length === 0;
  };

  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!validateForm()) return;

    setLoading(true);

    try {
      const response = await authAPI.login({
        email: formData.email,
        password: formData.password
      });

      setAuthToken(response.data.access_token);
      onSigninSuccess && onSigninSuccess();
    } catch (error) {
      console.error('Login error:', error);
      setErrors({ general: 'Invalid email or password. Please try again.' });
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="signin-form-container">
      <h2>Sign In to Your Account</h2>
      {errors.general && <div className="error-message">{errors.general}</div>}

      <form onSubmit={handleSubmit} className="auth-form">
        <div className="form-group">
          <label htmlFor="email">Email</label>
          <input
            type="email"
            id="email"
            name="email"
            value={formData.email}
            onChange={handleChange}
            className={errors.email ? 'error' : ''}
          />
          {errors.email && <div className="error-text">{errors.email}</div>}
        </div>

        <div className="form-group">
          <label htmlFor="password">Password</label>
          <input
            type="password"
            id="password"
            name="password"
            value={formData.password}
            onChange={handleChange}
            className={errors.password ? 'error' : ''}
          />
          {errors.password && <div className="error-text">{errors.password}</div>}
        </div>

        <button type="submit" disabled={loading} className="submit-button">
          {loading ? 'Signing In...' : 'Sign In'}
        </button>
      </form>
    </div>
  );
};

export default SigninForm;
```

### 4.5 Create Personalization Button Component
```jsx
// website/src/components/Auth/PersonalizationButton.jsx
import React, { useState, useEffect } from 'react';
import { authAPI } from '../../utils/auth';
import './Auth.css';

const PersonalizationButton = ({ onPersonalizationChange }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [settings, setSettings] = useState({
    difficulty_level: 'match_profile',
    content_depth: 'detailed',
    example_preference: 'practical'
  });
  const [loading, setLoading] = useState(false);
  const [isAuthenticated, setIsAuthenticated] = useState(false);

  useEffect(() => {
    // Check if user is authenticated
    const token = localStorage.getItem('auth_token');
    setIsAuthenticated(!!token);

    // Load current settings if authenticated
    if (token) {
      loadPersonalizationSettings();
    }
  }, []);

  const loadPersonalizationSettings = async () => {
    try {
      const response = await authAPI.getPersonalizationSettings();
      if (response.data) {
        setSettings(prev => ({ ...prev, ...response.data }));
      }
    } catch (error) {
      console.error('Error loading personalization settings:', error);
    }
  };

  const handleSave = async () => {
    setLoading(true);
    try {
      await authAPI.updatePersonalizationSettings(settings);
      onPersonalizationChange && onPersonalizationChange(settings);
      setIsOpen(false);
    } catch (error) {
      console.error('Error saving personalization settings:', error);
    } finally {
      setLoading(false);
    }
  };

  if (!isAuthenticated) {
    return (
      <div className="personalization-prompt">
        <p>Sign in to personalize your learning experience!</p>
        <a href="/auth/signin" className="auth-link">Sign In</a>
      </div>
    );
  }

  return (
    <div className="personalization-container">
      <button
        className="personalization-button"
        onClick={() => setIsOpen(!isOpen)}
      >
        {isOpen ? 'Close' : 'Personalize Content'}
      </button>

      {isOpen && (
        <div className="personalization-panel">
          <h3>Personalize Your Learning</h3>

          <div className="form-group">
            <label>Difficulty Level:</label>
            <select
              value={settings.difficulty_level}
              onChange={(e) => setSettings(prev => ({ ...prev, difficulty_level: e.target.value }))}
            >
              <option value="below_profile">Easier than my level</option>
              <option value="match_profile">Match my level</option>
              <option value="above_profile">More challenging</option>
            </select>
          </div>

          <div className="form-group">
            <label>Content Depth:</label>
            <select
              value={settings.content_depth}
              onChange={(e) => setSettings(prev => ({ ...prev, content_depth: e.target.value }))}
            >
              <option value="overview">Overview</option>
              <option value="detailed">Detailed</option>
              <option value="comprehensive">Comprehensive</option>
            </select>
          </div>

          <div className="form-group">
            <label>Example Preference:</label>
            <select
              value={settings.example_preference}
              onChange={(e) => setSettings(prev => ({ ...prev, example_preference: e.target.value }))}
            >
              <option value="theoretical">Theoretical</option>
              <option value="practical">Practical</option>
              <option value="both">Both</option>
            </select>
          </div>

          <div className="button-group">
            <button onClick={handleSave} disabled={loading}>
              {loading ? 'Saving...' : 'Save Settings'}
            </button>
            <button onClick={() => setIsOpen(false)}>Cancel</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationButton;
```

### 4.6 Create Auth CSS
```css
/* website/src/css/auth.css */
.auth-form {
  max-width: 600px;
  margin: 0 auto;
  padding: 2rem;
  border: 1px solid #ddd;
  border-radius: 8px;
  background-color: #fff;
}

.form-group {
  margin-bottom: 1.5rem;
}

.form-group label {
  display: block;
  margin-bottom: 0.5rem;
  font-weight: bold;
}

.form-group input,
.form-group select {
  width: 100%;
  padding: 0.75rem;
  border: 1px solid #ccc;
  border-radius: 4px;
  font-size: 1rem;
}

.form-group input.error {
  border-color: #e74c3c;
}

.error-text {
  color: #e74c3c;
  font-size: 0.875rem;
  margin-top: 0.25rem;
}

.error-message {
  background-color: #ffebee;
  color: #c62828;
  padding: 0.75rem;
  border-radius: 4px;
  margin-bottom: 1rem;
  border: 1px solid #ffcdd2;
}

.form-row {
  display: flex;
  gap: 1rem;
}

.form-row .form-group {
  flex: 1;
}

.form-section {
  margin: 2rem 0;
  padding: 1.5rem;
  border: 1px solid #eee;
  border-radius: 8px;
  background-color: #fafafa;
}

.form-section h3 {
  margin-top: 0;
  margin-bottom: 1rem;
}

.skills-category {
  margin-bottom: 1.5rem;
}

.skills-category h4 {
  margin-bottom: 0.75rem;
}

.skill-item {
  display: flex;
  gap: 0.5rem;
  align-items: center;
  margin-bottom: 0.5rem;
}

.skill-item input,
.skill-item select {
  flex: 1;
}

.skill-item button {
  background-color: #e74c3c;
  color: white;
  border: none;
  padding: 0.25rem 0.5rem;
  border-radius: 4px;
  cursor: pointer;
}

.submit-button {
  width: 100%;
  padding: 1rem;
  background-color: #3498db;
  color: white;
  border: none;
  border-radius: 4px;
  font-size: 1.1rem;
  cursor: pointer;
}

.submit-button:disabled {
  background-color: #bdc3c7;
  cursor: not-allowed;
}

.personalization-container {
  position: relative;
  display: inline-block;
}

.personalization-button {
  background-color: #2ecc71;
  color: white;
  border: none;
  padding: 0.5rem 1rem;
  border-radius: 4px;
  cursor: pointer;
  font-size: 0.9rem;
}

.personalization-panel {
  position: absolute;
  top: 100%;
  right: 0;
  background-color: white;
  border: 1px solid #ddd;
  border-radius: 8px;
  padding: 1.5rem;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
  z-index: 1000;
  min-width: 300px;
}

.personalization-panel h3 {
  margin-top: 0;
  margin-bottom: 1rem;
}

.button-group {
  display: flex;
  gap: 0.5rem;
  margin-top: 1rem;
}

.button-group button {
  flex: 1;
  padding: 0.5rem;
  border: 1px solid #ddd;
  border-radius: 4px;
  cursor: pointer;
}

.button-group button:first-child {
  background-color: #3498db;
  color: white;
}

.personalization-prompt {
  background-color: #e3f2fd;
  padding: 1rem;
  border-radius: 8px;
  text-align: center;
}

.personalization-prompt .auth-link {
  display: inline-block;
  margin-top: 0.5rem;
  padding: 0.5rem 1rem;
  background-color: #3498db;
  color: white;
  text-decoration: none;
  border-radius: 4px;
}