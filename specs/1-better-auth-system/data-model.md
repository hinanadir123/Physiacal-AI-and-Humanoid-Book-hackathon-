# Data Model: Better-Auth Authentication System

## User Entity

**Fields:**
- `id` (UUID, primary key): Unique identifier for the user
- `email` (string, unique, not null): User's email address for authentication
- `hashed_password` (string, not null): Securely hashed password
- `created_at` (timestamp, not null): Account creation timestamp
- `updated_at` (timestamp, not null): Last update timestamp
- `is_active` (boolean, default: true): Account active status
- `email_verified` (boolean, default: false): Email verification status
- `is_onboarded` (boolean, default: false): Whether user has completed onboarding

## UserProfile Entity

**Fields:**
- `user_id` (UUID, primary key, foreign key to User.id): Reference to the user
- `first_name` (string): User's first name
- `last_name` (string): User's last name
- `experience_level` (enum: 'beginner', 'intermediate', 'advanced'): User's experience level
- `software_skills` (JSONB): Structured data about user's software skills
  - `programming_languages`: Array of objects with name and proficiency (1-5)
  - `frameworks`: Array of objects with name and proficiency (1-5)
  - `tools`: Array of objects with name and proficiency (1-5)
- `hardware_access` (JSONB): Information about user's hardware access
  - `development_kits`: Array of strings (e.g., "Raspberry Pi", "Arduino")
  - `robotics_platforms`: Array of strings (e.g., "ROS 2", "NVIDIA Isaac")
  - `simulation_environments`: Array of strings (e.g., "Gazebo", "Unity")
- `learning_preferences` (JSONB): User's learning preferences
  - `content_types`: Array of strings (e.g., "text", "video", "interactive")
  - `learning_pace`: String ("slow", "moderate", "fast")
  - `preferred_topics`: Array of strings (e.g., "ROS", "Computer Vision", "Path Planning")
- `personalization_settings` (JSONB): Settings for content personalization
  - `difficulty_level`: String ("match_profile", "above_profile", "below_profile")
  - `content_depth`: String ("overview", "detailed", "comprehensive")
  - `example_preference`: String ("theoretical", "practical", "both")
- `created_at` (timestamp, not null): Profile creation timestamp
- `updated_at` (timestamp, not null): Last profile update timestamp

## Validation Rules

**User Entity:**
- Email must be valid email format (RFC 5322)
- Password must meet strength requirements (min 8 chars, 1 uppercase, 1 lowercase, 1 number, 1 special char)
- Email must be unique across all users
- Email cannot be changed after account creation

**UserProfile Entity:**
- Experience level must be one of the defined enum values
- Software skills must be properly structured JSON with proficiency values 1-5
- Hardware access must be properly structured JSON with valid hardware names
- Profile must be linked to an existing user
- All JSONB fields must be valid JSON structures

## State Transitions

**User Account:**
- `inactive` → `active`: Upon successful email verification
- `active` → `suspended`: Upon admin action or security concerns
- `suspended` → `active`: Upon admin action
- `active` → `deleted`: Upon user request (soft delete)

## Relationships

- One User → One UserProfile (1:1 relationship)
- UserProfile depends on User for existence
- All profile operations require active User account