"""
Smoke test for RAG Chatbot functionality
"""
import requests
import json
import time

BASE_URL = "http://127.0.0.1:5000"

def test_health():
    """Test health endpoint"""
    print("Testing health endpoint...")
    response = requests.get(f"{BASE_URL}/health")
    assert response.status_code == 200
    data = response.json()
    assert data['status'] == 'healthy'
    print("[PASS] Health endpoint working")

def test_list_collections():
    """Test collections endpoint"""
    print("Testing collections endpoint...")
    response = requests.get(f"{BASE_URL}/collections")
    assert response.status_code == 200
    data = response.json()
    assert 'collections' in data
    assert len(data['collections']) > 0
    print(f"[PASS] Found collections: {data['collections']}")

def test_collection_info():
    """Test specific collection info"""
    print("Testing collection info endpoint...")
    response = requests.get(f"{BASE_URL}/collections/rag_embedding")
    assert response.status_code == 200
    data = response.json()
    assert data['name'] == 'rag_embedding'
    assert data['status'] == 'available'
    print(f"[PASS] Collection info: {data}")

def test_chat_functionality():
    """Test chat functionality with fallback"""
    print("Testing chat functionality...")
    payload = {
        "query": "What is voice to action in robotics?",
        "llm": "claude"
    }
    response = requests.post(f"{BASE_URL}/chat",
                           json=payload,
                           headers={"Content-Type": "application/json"})
    assert response.status_code == 200
    data = response.json()
    assert 'response' in data
    assert 'llm_used' in data
    assert 'Found relevant information' in data['response'] or 'No relevant information' in data['response']
    print(f"[PASS] Chat response received (using {data['llm_used']})")

def run_all_tests():
    """Run all smoke tests"""
    print("Starting RAG Chatbot smoke tests...\n")

    try:
        test_health()
        print()

        test_list_collections()
        print()

        test_collection_info()
        print()

        test_chat_functionality()
        print()

        print("[SUCCESS] All smoke tests passed! RAG system is working correctly.")

    except Exception as e:
        print(f"[ERROR] Test failed: {str(e)}")
        raise

if __name__ == "__main__":
    run_all_tests()