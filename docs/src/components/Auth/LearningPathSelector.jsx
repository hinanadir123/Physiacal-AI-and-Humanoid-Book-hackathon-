import React, { useState, useEffect } from 'react';
import { authAPI, isAuthenticated } from '../../utils/auth';

const LearningPathSelector = () => {
  const [selectedPath, setSelectedPath] = useState('auto'); // auto, beginner, intermediate, advanced
  const [userProfile, setUserProfile] = useState(null);
  const [loading, setLoading] = useState(true);

  useEffect(() => {
    loadUserProfile();
  }, []);

  const loadUserProfile = async () => {
    if (isAuthenticated()) {
      try {
        const response = await authAPI.getCurrentProfile();
        setUserProfile(response.data);
        // Set default path based on user's experience level
        if (response.data.experience_level) {
          setSelectedPath(response.data.experience_level);
        }
      } catch (error) {
        console.error('Error loading profile:', error);
      }
    }
    setLoading(false);
  };

  const handlePathChange = async (path) => {
    setSelectedPath(path);

    // Update user's experience preference in their profile
    if (isAuthenticated()) {
      try {
        await authAPI.updateProfile({ experience_level: path });
      } catch (error) {
        console.error('Error updating experience level:', error);
      }
    }
  };

  if (loading) {
    return <div className="learning-path-loading">Loading learning path...</div>;
  }

  return (
    <div className="learning-path-selector">
      <h3>Select Your Learning Path</h3>
      <div className="path-options">
        <button
          className={`path-option ${selectedPath === 'beginner' ? 'active' : ''}`}
          onClick={() => handlePathChange('beginner')}
        >
          <div className="path-icon">🌱</div>
          <h4>Beginner Path</h4>
          <p>Start with fundamentals and basic concepts</p>
        </button>

        <button
          className={`path-option ${selectedPath === 'intermediate' ? 'active' : ''}`}
          onClick={() => handlePathChange('intermediate')}
        >
          <div className="path-icon">🌿</div>
          <h4>Intermediate Path</h4>
          <p>Build on existing knowledge with more complex topics</p>
        </button>

        <button
          className={`path-option ${selectedPath === 'advanced' ? 'active' : ''}`}
          onClick={() => handlePathChange('advanced')}
        >
          <div className="path-icon">🚀</div>
          <h4>Advanced Path</h4>
          <p>Deep dive into advanced concepts and applications</p>
        </button>

        <button
          className={`path-option ${selectedPath === 'auto' ? 'active' : ''}`}
          onClick={() => handlePathChange('auto')}
        >
          <div className="path-icon">🤖</div>
          <h4>Auto-Adapt</h4>
          <p>System adapts based on your progress and profile</p>
        </button>
      </div>

      <div className="current-path">
        <p>Current Path: <strong>{selectedPath.charAt(0).toUpperCase() + selectedPath.slice(1)}</strong></p>
      </div>
    </div>
  );
};

export default LearningPathSelector;