# OpenAPI Contract: Better-Auth Authentication System

## Authentication API

### Register User with Profile Data
- **Endpoint**: `POST /api/v1/auth/register`
- **Description**: Register a new user with authentication credentials and profile data
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "SecurePassword123!",
    "first_name": "John",
    "last_name": "Doe",
    "experience_level": "intermediate",
    "software_skills": {
      "programming_languages": [
        {"name": "Python", "proficiency": 4},
        {"name": "JavaScript", "proficiency": 3}
      ],
      "frameworks": [
        {"name": "FastAPI", "proficiency": 3},
        {"name": "React", "proficiency": 2}
      ],
      "tools": [
        {"name": "Docker", "proficiency": 3},
        {"name": "Git", "proficiency": 4}
      ]
    },
    "hardware_access": {
      "development_kits": ["Raspberry Pi", "Arduino"],
      "robotics_platforms": ["ROS 2", "NVIDIA Isaac"],
      "simulation_environments": ["Gazebo", "Unity"]
    },
    "learning_preferences": {
      "content_types": ["text", "interactive"],
      "learning_pace": "moderate",
      "preferred_topics": ["ROS", "Computer Vision"]
    }
  }
  ```
- **Response**:
  - Success (201): User created with access token
  - Error (400): Invalid input data
  - Error (409): Email already exists

### User Login
- **Endpoint**: `POST /api/v1/auth/login`
- **Description**: Authenticate user and return access token
- **Request Body**:
  ```json
  {
    "email": "user@example.com",
    "password": "SecurePassword123!"
  }
  ```
- **Response**:
  - Success (200): Access and refresh tokens
  - Error (401): Invalid credentials

### User Logout
- **Endpoint**: `POST /api/v1/auth/logout`
- **Description**: Logout user and invalidate session
- **Authentication**: Bearer token required
- **Response**:
  - Success (200): User logged out successfully
  - Error (401): Unauthorized

### Get Current User Profile
- **Endpoint**: `GET /api/v1/profiles/me`
- **Description**: Get current user's profile data
- **Authentication**: Bearer token required
- **Response**:
  - Success (200): User profile data
  - Error (401): Unauthorized

### Update User Profile
- **Endpoint**: `PUT /api/v1/profiles/me`
- **Description**: Update current user's profile data
- **Authentication**: Bearer token required
- **Request Body**: Profile data to update (partial updates allowed)
- **Response**:
  - Success (200): Updated profile data
  - Error (400): Invalid input data
  - Error (401): Unauthorized

### Update Personalization Settings
- **Endpoint**: `PUT /api/v1/profiles/me/personalization`
- **Description**: Update personalization settings for content customization
- **Authentication**: Bearer token required
- **Request Body**:
  ```json
  {
    "difficulty_level": "match_profile",
    "content_depth": "detailed",
    "example_preference": "practical"
  }
  ```
- **Response**:
  - Success (200): Updated personalization settings
  - Error (400): Invalid input data
  - Error (401): Unauthorized

### Get Personalization Settings
- **Endpoint**: `GET /api/v1/profiles/me/personalization`
- **Description**: Get current user's personalization settings
- **Authentication**: Bearer token required
- **Response**:
  - Success (200): User's personalization settings
  - Error (401): Unauthorized

## Security Schemes
- **BearerAuth**: Bearer token authentication for protected endpoints
- **Request Signing**: All requests must include appropriate authentication

## Error Responses
All error responses follow the standard format:
```json
{
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": "Additional error details if applicable"
  }
}
```

## Rate Limiting
- Authentication endpoints: 10 requests per minute per IP
- Profile endpoints: 100 requests per minute per authenticated user