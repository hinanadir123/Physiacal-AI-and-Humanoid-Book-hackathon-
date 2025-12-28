from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from auth.routes import router as auth_router
from api.v1.profiles import router as profile_router
from models import init_db

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
async def on_startup():
    init_db()

@app.get("/")
def read_root():
    return {"message": "Better-Auth API is running!"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run("app:app", host="0.0.0.0", port=8000, reload=True)