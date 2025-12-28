import pytest
from fastapi.testclient import TestClient
from app import app
from models.database import get_db
from models.user import User, UserProfile
from auth.utils import create_access_token
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Use an in-memory SQLite database for testing
SQLALCHEMY_DATABASE_URL = "sqlite:///./test.db"

engine = create_engine(
    SQLALCHEMY_DATABASE_URL, connect_args={"check_same_thread": False}
)
TestingSessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)


# Dependency override for testing
def override_get_db():
    try:
        db = TestingSessionLocal()
        yield db
    finally:
        db.close()


app.dependency_overrides[get_db] = override_get_db

client = TestClient(app)

def test_get_profile():
    # First create and login a user to get a token
    register_response = client.post(
        "/api/v1/auth/register",
        json={
            "email": "profile@example.com",
            "password": "testpassword123",
            "first_name": "Profile",
            "last_name": "Test",
            "experience_level": "intermediate"
        }
    )

    login_response = client.post(
        "/api/v1/auth/login",
        json={
            "email": "profile@example.com",
            "password": "testpassword123"
        }
    )

    token = login_response.json()["access_token"]

    # Now try to get the profile
    response = client.get(
        "/api/v1/profiles/me",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    data = response.json()
    assert "user_id" in data
    assert data["first_name"] == "Profile"

def test_update_profile():
    # First create and login a user to get a token
    client.post(
        "/api/v1/auth/register",
        json={
            "email": "update@example.com",
            "password": "testpassword123",
            "first_name": "Update",
            "last_name": "Test",
            "experience_level": "beginner"
        }
    )

    login_response = client.post(
        "/api/v1/auth/login",
        json={
            "email": "update@example.com",
            "password": "testpassword123"
        }
    )

    token = login_response.json()["access_token"]

    # Update the profile
    response = client.put(
        "/api/v1/profiles/me",
        json={
            "first_name": "Updated",
            "experience_level": "advanced"
        },
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["first_name"] == "Updated"
    assert data["experience_level"] == "advanced"

def test_get_personalization_settings():
    # First create and login a user to get a token
    client.post(
        "/api/v1/auth/register",
        json={
            "email": "personalize@example.com",
            "password": "testpassword123",
            "first_name": "Personalize",
            "last_name": "Test",
            "experience_level": "intermediate"
        }
    )

    login_response = client.post(
        "/api/v1/auth/login",
        json={
            "email": "personalize@example.com",
            "password": "testpassword123"
        }
    )

    token = login_response.json()["access_token"]

    # Get personalization settings
    response = client.get(
        "/api/v1/profiles/me/personalization",
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    # Should return an empty dict or default settings

def test_update_personalization_settings():
    # First create and login a user to get a token
    client.post(
        "/api/v1/auth/register",
        json={
            "email": "personalize2@example.com",
            "password": "testpassword123",
            "first_name": "Personalize2",
            "last_name": "Test",
            "experience_level": "intermediate"
        }
    )

    login_response = client.post(
        "/api/v1/auth/login",
        json={
            "email": "personalize2@example.com",
            "password": "testpassword123"
        }
    )

    token = login_response.json()["access_token"]

    # Update personalization settings
    response = client.put(
        "/api/v1/profiles/me/personalization",
        json={
            "difficulty_level": "match_profile",
            "content_depth": "detailed",
            "example_preference": "practical"
        },
        headers={"Authorization": f"Bearer {token}"}
    )
    assert response.status_code == 200
    data = response.json()
    assert data["difficulty_level"] == "match_profile"