import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { Navbar } from '@docusaurus/theme-classic';
import { isAuthenticated, getAuthToken } from '../../utils/auth';
import PersonalizationButton from '../Auth/PersonalizationButton';

const CustomNavbar = () => {
  const [isAuth, setIsAuth] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const checkAuth = () => {
      const authStatus = isAuthenticated();
      setIsAuth(authStatus);
    };

    checkAuth();

    // Listen for storage events to update auth status across tabs
    const handleStorageChange = () => {
      checkAuth();
    };

    window.addEventListener('storage', handleStorageChange);
    return () => window.removeEventListener('storage', handleStorageChange);
  }, []);

  return (
    <Navbar>
      {/* Personalization button only shows when authenticated */}
      {isAuth && (
        <div style={{ marginLeft: '10px' }}>
          <PersonalizationButton />
        </div>
      )}
    </Navbar>
  );
};

export default CustomNavbar;