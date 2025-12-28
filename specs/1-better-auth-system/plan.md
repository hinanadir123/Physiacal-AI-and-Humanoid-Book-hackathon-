# Implementation Plan: Better-Auth Authentication System

**Branch**: `1-better-auth-system` | **Date**: 2025-12-25 | **Spec**: [link](specs/1-better-auth-system/spec.md)
**Input**: Feature specification from `/specs/[###-feature-name]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a Better-Auth-based authentication system integrated into the Docusaurus textbook project. The system will collect structured profile data about users' software skills, hardware access, and experience level during registration, and use this data to personalize chapter content for logged-in users via a personalization button.

## Technical Context

**Language/Version**: JavaScript/TypeScript for frontend, Python 3.11 for backend API
**Primary Dependencies**: Better-Auth library, Docusaurus, PostgreSQL database
**Storage**: PostgreSQL database for user profiles and authentication data
**Testing**: Jest for frontend components, pytest for backend API
**Target Platform**: Docusaurus-based textbook website (Linux server deployment)
**Project Type**: Web
**Performance Goals**: Support 1000 concurrent users, sub-200ms authentication response times
**Constraints**: <200ms p95 authentication, secure handling of credentials, GDPR compliance for user data, seamless integration with existing Docusaurus structure
**Scale/Scope**: 10k users initially, scalable to 100k users

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the project constitution, this authentication system must:
- Support the overall Physical AI & Humanoid Robotics educational platform
- Integrate with the Docusaurus-based technical book system
- Support the RAG chatbot functionality by providing user context
- Align with the vision of connecting digital intelligence to physical robotic behavior through personalized learning

No constitution violations identified for this feature.

## Project Structure

### Documentation (this feature)

```text
specs/1-better-auth-system/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (integrated into existing textbook project)

```text
# Backend API (new)
backend/
├── app.py              # FastAPI application with Better-Auth
├── models/
│   ├── __init__.py
│   ├── user.py         # User and profile models
│   └── database.py     # Database configuration
├── auth/
│   ├── __init__.py
│   ├── routes.py       # Authentication routes
│   ├── schemas.py      # Pydantic schemas
│   └── utils.py        # Authentication utilities
├── api/
│   ├── __init__.py
│   └── v1/             # API v1 routes
│       ├── __init__.py
│       ├── auth.py     # Auth endpoints
│       └── profiles.py # Profile endpoints
└── tests/
    ├── __init__.py
    ├── test_auth.py
    └── test_profiles.py

# Docusaurus Frontend (existing + additions)
website/
├── src/
│   ├── components/
│   │   ├── Auth/
│   │   │   ├── SignupForm.jsx      # Signup form with background questions
│   │   │   ├── SigninForm.jsx      # Signin form
│   │   │   ├── ProfileEditor.jsx   # Profile editing component
│   │   │   └── PersonalizationButton.jsx  # Personalization button for chapters
│   │   ├── User/
│   │   │   └── UserProfile.jsx     # User profile display
│   │   └── Common/
│   │       └── ProtectedRoute.jsx  # Route protection component
│   ├── pages/
│   │   ├── auth/
│   │   │   ├── signup.jsx
│   │   │   ├── signin.jsx
│   │   │   └── profile.jsx
│   │   └── ...
│   ├── utils/
│   │   ├── auth.js                 # Authentication utilities
│   │   └── api.js                  # API client
│   └── css/
│       └── auth.css                # Authentication component styles
├── docusaurus.config.js            # Updated config with auth routes
├── package.json                    # Updated dependencies
└── static/
    └── img/
        └── auth/                   # Authentication-related images

# Database
├── migrations/                     # Database migration files
│   ├── 001_create_users_table.py
│   └── 002_create_profiles_table.py
└── requirements.txt               # Backend dependencies
```

**Structure Decision**: Integrate authentication system into existing Docusaurus textbook project with a separate backend API for authentication services. Frontend components are added to the existing Docusaurus structure to maintain consistency with the textbook interface.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|