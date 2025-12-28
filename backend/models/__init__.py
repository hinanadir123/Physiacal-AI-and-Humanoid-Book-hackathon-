from .database import Base, engine
from .user import User, UserProfile

def init_db():
    """Initialize the database tables"""
    Base.metadata.create_all(bind=engine)