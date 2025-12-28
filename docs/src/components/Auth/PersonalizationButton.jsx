import React, { useState, useEffect } from 'react';
import { authAPI } from '../../utils/auth';
import LearningPathSelector from './LearningPathSelector';
import '../../css/auth.css';

const PersonalizationButton = ({ onPersonalizationChange }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [settings, setSettings] = useState({
    difficulty_level: 'match_profile',
    content_depth: 'detailed',
    example_preference: 'practical'
  });
  const [activeTab, setActiveTab] = useState('preferences'); // preferences or paths
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
          <div className="personalization-tabs">
            <button
              className={`tab-button ${activeTab === 'preferences' ? 'active' : ''}`}
              onClick={() => setActiveTab('preferences')}
            >
              Preferences
            </button>
            <button
              className={`tab-button ${activeTab === 'paths' ? 'active' : ''}`}
              onClick={() => setActiveTab('paths')}
            >
              Learning Paths
            </button>
          </div>

          {activeTab === 'preferences' && (
            <>
              <h3>Content Preferences</h3>

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
            </>
          )}

          {activeTab === 'paths' && (
            <LearningPathSelector />
          )}
        </div>
      )}
    </div>
  );
};

export default PersonalizationButton;